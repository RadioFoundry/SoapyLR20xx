// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Spencer Fowler
/*
 * The link over TCP (LR20XX_LINK_TCP_PORT): one host at a time, a new one replaces the old. And UDP discovery
 * (LR20XX_LINK_DISCOVERY_PORT): a datagram starting "LR20XX?" is answered with an RSP frame, sequence 0, error 0,
 * whose answer is the HELLO's (lr20xx_link_put_hello, LR20XX_LINK_HELLO_SIZE bytes).
 */
#include <errno.h>
#include <string.h>

#include "bridge.h"
#include "links.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

static const char *TAG = "link";

EXT_RAM_BSS_ATTR static bridge_link_t tcp_link;
static int listen_fd = -1;

/* The host has sent something not yet read. */
static bool tcp_pending(bridge_link_t *l)
{
    const int fd = (int)(intptr_t)l->ctx;
    fd_set r;
    FD_ZERO(&r);
    FD_SET(fd, &r);
    struct timeval tv = {0, 0};
    return select(fd + 1, &r, NULL, NULL, &tv) > 0;
}

/* All of it sent, or -1: the host gone, 2 s with no room (a host that stopped reading), or a new host waiting to
 * replace this one (a host killed mid-stream leaves its window full: the next one isn't kept waiting on it) */
static int tcp_send(bridge_link_t *l, const uint8_t *data, size_t n)
{
    int fd = (int)(intptr_t)l->ctx;
    int64_t until = esp_timer_get_time() + 2000000;
    while (n) {
        int k = send(fd, data, n, MSG_DONTWAIT);
        if (k > 0) {
            data += k;
            n -= (size_t)k;
            until = esp_timer_get_time() + 2000000;
            continue;
        }
        if (k < 0 && errno == EINTR)
            continue;
        if (k == 0 || (errno != EAGAIN && errno != EWOULDBLOCK))
            return -1;
        const int64_t left = until - esp_timer_get_time();
        if (left <= 0)
            return -1;
        fd_set r, w;
        FD_ZERO(&r);
        FD_ZERO(&w);
        FD_SET(fd, &w);
        FD_SET(listen_fd, &r);
        struct timeval tv = {(time_t)(left / 1000000), (suseconds_t)(left % 1000000)};
        if (select((fd > listen_fd ? fd : listen_fd) + 1, &r, &w, NULL, &tv) > 0 && FD_ISSET(listen_fd, &r))
            return -1;
    }
    return 0;
}

static void client_close(int *fd)
{
    if (*fd < 0)
        return;
    bridge_link_closed(&tcp_link);
    shutdown(*fd, SHUT_RDWR);
    close(*fd);
    *fd = -1;
    ESP_LOGI(TAG, "host gone");
}

static void tcp_task(void *arg)
{
    int ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons(LR20XX_LINK_TCP_PORT), .sin_addr.s_addr = 0};
    listen_fd = ls;
    if (bind(ls, (struct sockaddr *)&a, sizeof a) || listen(ls, 1)) {
        ESP_LOGE(TAG, "TCP port %d: %s", LR20XX_LINK_TCP_PORT, strerror(errno));
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "listening on TCP %d", LR20XX_LINK_TCP_PORT);
    static uint8_t in[4096];
    int fd = -1;
    for (;;) {
        fd_set r;
        FD_ZERO(&r);
        FD_SET(ls, &r);
        if (fd >= 0)
            FD_SET(fd, &r);
        /* receiving: don't wait on the host, the samples are what's waited for (pump) */
        struct timeval tv = {0, bridge_idle_ms(50) * 1000};
        int nr = select((fd > ls ? fd : ls) + 1, &r, NULL, NULL, &tv);
        if (nr > 0 && FD_ISSET(ls, &r)) {
            struct sockaddr_in peer;
            socklen_t pl = sizeof peer;
            int c = accept(ls, (struct sockaddr *)&peer, &pl);
            if (c >= 0) {
                client_close(&fd); /* the new host replaces the old */
                fd = c;
                setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
                tcp_link.send = tcp_send;
                tcp_link.pending = tcp_pending;
                tcp_link.ctx = (void *)(intptr_t)fd;
                bridge_link_open(&tcp_link);
                ESP_LOGI(TAG, "host %s", inet_ntoa(peer.sin_addr));
            }
        }
        if (nr > 0 && fd >= 0 && FD_ISSET(fd, &r)) {
            int k = recv(fd, in, sizeof in, 0);
            if (k <= 0 || bridge_link_input(&tcp_link, in, (size_t)k) < 0) {
                client_close(&fd);
                continue;
            }
        }
        if (fd >= 0) {
            /* up to a few frames of samples, then back to the host's input (at once if it has sent some) */
            for (int i = 0; i < 4; i++) {
                int p = bridge_link_pump(&tcp_link, i ? 0 : 5);
                if (p < 0) {
                    client_close(&fd);
                    break;
                }
                if (!p || tcp_pending(&tcp_link))
                    break;
            }
        }
    }
}

static void discovery_task(void *arg)
{
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons(LR20XX_LINK_DISCOVERY_PORT),
                            .sin_addr.s_addr = 0};
    if (bind(s, (struct sockaddr *)&a, sizeof a)) {
        ESP_LOGE(TAG, "UDP port %d: %s", LR20XX_LINK_DISCOVERY_PORT, strerror(errno));
        vTaskDelete(NULL);
    }
    for (;;) {
        uint8_t q[32];
        struct sockaddr_in from;
        socklen_t fl = sizeof from;
        int k = recvfrom(s, q, sizeof q, 0, (struct sockaddr *)&from, &fl);
        if (k < 7 || memcmp(q, "LR20XX?", 7))
            continue;
        uint8_t hello[LR20XX_LINK_HELLO_SIZE], out[LR20XX_LINK_HEADER + 6 + LR20XX_LINK_HELLO_SIZE];
        size_t hn = bridge_hello(hello);
        size_t n = lr20xx_link_rsp(out, sizeof out, 0, 0, hello, hn);
        sendto(s, out, n, 0, (struct sockaddr *)&from, fl);
    }
}

void link_tcp_start(void)
{
    /* core 1: core 0 is the reader's and the network stack's */
    xTaskCreatePinnedToCore(tcp_task, "link_tcp", 6144, NULL, 10, NULL, LINK_CORE);
}

void link_discovery_start(void)
{
    xTaskCreatePinnedToCore(discovery_task, "discovery", 3072, NULL, 5, NULL, LINK_CORE);
}
