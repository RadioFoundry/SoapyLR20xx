<!-- SPDX-License-Identifier: MIT -->
<!-- SPDX-FileCopyrightText: 2026 Spencer Fowler -->
# LR20xx bridge firmware

An ESP32-S3 app that turns a board with an LR20xx on its SPI bus into a network SDR. The chip's 1 KB receive
FIFO lasts 2 ms, too short for a host on the far side of Wi-Fi or USB. So the ESP32 reads it (`lr20xx_iq_poll`)
and streams the samples. The host configures, receives and sends transmissions over the bridge link
(`proto/include/lr20xx_link.h`, described in `docs/link.md`). SoapyLR20xx's bridge transport is the host side.

Status: runs on the Meshnology W12 (2026-10-26). Over USB, as a network adapter or a serial port, every receive
rate streams without loss (below). Transmit and Wi-Fi streaming are not yet tried on the board.

## How it works

- **Reader task:** on core 0 at priority 24, above Wi-Fi. Under Wi-Fi's priority, Wi-Fi traffic kept a reader off
  the CPU past the FIFO's 2 ms. It reads the chip and puts each read into a message buffer in PSRAM: a header and
  the samples. When that buffer is full, the read is dropped and the next one is flagged as a gap.
- **Link task:** on core 1. It handles the host's commands and sends the buffered reads as `RX_DATA` frames, with
  up to 4 KB of samples a frame.
- **Shared chip:** commands and the reader take turns on one mutex. The reader gives a command a tick when one is
  waiting.
- **Transmit:** the host sends the whole transmission first (`TX_BEGIN`, `TX_DATA`, `TX_COMMIT`). It runs on the
  reader's task, so the TX FIFO stays fed. A receive session is paused for it and then resumed.
- **Format:** samples go out at the chip's rate, cs16 or cs8, packed to the narrowest width that holds each
  frame's values (`docs/link.md`). Nothing is lost or resampled; int8 takes 6 bits, int16 noise about 10.

## Building

ESP-IDF v5.5, from this folder:

```sh
. ~/esp/v5.5/esp-idf/export.sh
idf.py -B ../build/fw-w12 -D SDKCONFIG=../build/fw-w12/sdkconfig \
       -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.w12" set-target esp32s3 build
```

`sdkconfig.defaults` is for an ESP32-S3 with 16 MB flash and 8 MB octal PSRAM. `sdkconfig.defaults.w12` sets
the W12's pins. For another board, copy it and change the pins and `CONFIG_LR20XX_BOARD`.

The link is TCP over Wi-Fi by default. Add one overlay last for USB:

| Overlay | The host sees | For |
|---|---|---|
| `sdkconfig.defaults.ncm` | a USB network adapter, plus a CDC console port | the S3: TCP, discovery and HTTP over USB; Wi-Fi stays on |
| `sdkconfig.defaults.jtag` | the USB Serial/JTAG port as the link | any chip with one (C3, C6, H2, S3); flashing over it still works |
| `sdkconfig.defaults.usb` | a TinyUSB CDC port as the link | the S2 and S3 |

A UART link (through a USB-UART bridge, or to a host's own UART) is the "Serial" link with the UART port in
menuconfig. With a serial link, the console and the log move to UART0, since their text would break the frames.

Measured on the W12 to a Mac, 60 s at each rate, AGC on (2026-10-26):

| Link | cs16 125 k | cs16 250 k | cs8 400 k | cs8 576 k | Buffer peak |
|---|---|---|---|---|---|
| USB network (NCM) | 100 % | 100 % | 100 % | 100 % | 5.8 KB |
| USB Serial/JTAG (8 s each) | 100 % | 99.6 % | 100 % | 99.9 % | 0.7 KB |

Full-speed USB carries about 1 MB/s in all. Packing is what fits 250 kS/s cs16 (1 MB/s unpacked) and 576 kS/s cs8;
without it they ran at 92 % and 81 %. The NCM link also needs its larger NTB buffers (`sdkconfig.defaults.ncm`).

## Settings (menuconfig, "LR20xx bridge")

| Option | Default (W12) | What |
|---|---|---|
| `LR20XX_BOARD` | `w12` | The board around the chip, as `lr20xx_board_parse` reads it (`"generic tcxo=3 irq=7"`) |
| `LR20XX_ALLOW_UNTESTED` | off | Run raw I/Q on a ROM not in the library's table |
| Pins | NSS 8, SCK 9, MOSI 10, MISO 11, NRESET 12, BUSY 13, IRQ 7, PA_EN 4; SPI2 at 16 MHz | The IRQ GPIO is the one the board's IRQ DIO drives (-1 for none: the reader polls the FIFO level) |
| `LR20XX_RING_KB` | 1024 | The sample buffer, in PSRAM: 1.8 s at 576 kS/s int8 |
| `LR20XX_TX_BUF_KB` | 2048 | The longest transmission: 1 M I/Q samples, 2 M frequency samples |
| Link to the host | TCP | TCP (Wi-Fi, and the USB network if it's on), or Serial |
| `LR20XX_USB_NCM` | off | The network over USB as well (S3) |
| `LR20XX_USB_NCM_IP` | empty | The bridge's USB address: empty for a /30 in 10.73.0.0/16 picked from the MAC |
| Serial port | USB Serial/JTAG | USB Serial/JTAG, a UART (number, pins, baud: 2 Mbaud by default), or TinyUSB CDC |
| `LR20XX_HOSTNAME` | `lr20xx-bridge` | |
| `LR20XX_HTTP` | on | `GET /api/status`, `POST /api/ota` |

## Flashing

Over USB. A Wi-Fi or USB Serial/JTAG build leaves the port to the ROM's reset, so `idf.py flash` just works. A
TinyUSB build (the USB network, or a CDC link) has the port to itself: send `download` on its console first, or hold
BOOT while plugging in. Then:

```sh
idf.py -B ../build/fw-w12 -p /dev/cu.usbmodem<N> flash monitor
```

Once it runs, over Wi-Fi or the USB network:

```sh
curl -X POST --data-binary @../build/fw-w12/lr20xx_bridge.bin http://<board>/api/ota
```

It answers, then restarts a second later. Receiving pauses while the image is written. `GET /api/status` gives
`uptime_s` (it drops when the new image is running), the reset reason, the firmware version and the bridge's
counts.

## Wi-Fi

The network is kept in NVS. Set it on the serial console (the USB serial port on the W12):

```
lr20xx> wifi MyNetwork MyPassword
```

`status` prints the address, the chip and the counts. `restart` restarts. Power save is off, because it holds
packets until the next beacon.

With the USB network, the console is the CDC port beside it (`/dev/cu.usbmodem…`, `/dev/ttyACM…`), and `download`
restarts into the ROM's download mode, so `idf.py flash` works without the BOOT button.

## Transports

- **TCP:** port 7373 (`LR20XX_LINK_TCP_PORT`), one host at a time. A new connection replaces the old one. When a
  host goes away, receiving stops and any buffered transmission is dropped.
- **Discovery:** UDP port 7374 (`LR20XX_LINK_DISCOVERY_PORT`). A datagram starting with `LR20XX?` gets an answer
  sent back to its source address and port: one link frame, an `RSP` with sequence 0 and error 0, carrying the
  `HELLO` answer (`lr20xx_link_put_hello`, 106 bytes). That's 116 bytes in all. It's answered with either link.
- **USB network** (NCM, optional): the bridge's own /30 in 10.73.0.0/16, picked from its MAC, so several bridges
  on one host don't share a subnet. Its DHCP server gives the host the other address and no router or DNS, so the
  host's default route stays where it was. The console's `status` gives both addresses; discovery finds it.
- **Serial** (optional): a HELLO starts a session. USB Serial/JTAG and TinyUSB ports end one when the host lets go
  of the port; a UART can't tell, so its session lasts until the next HELLO. A UART needs the module's `baud=`.

`HELLO` gives:

- the link version
- the ROM version, and whether the library knows it
- what the device can do: receive, transmit, IRQ. If the chip didn't come up this is 0, and an `EVENT` with the
  error follows the answer.
- the longest transmission, counted in 2-byte I/Q samples
- the board's name, frequency range and maximum power
- the firmware version
- a serial number taken from the Wi-Fi MAC

`READ_REG`, `WRITE_REG`, `RAW` and `TEMP` are refused while receiving, with `LR20XX_ERR_INVALID_STATE`. `RAW`'s two
status bytes are the command status (0 FAIL, 1 PERR, 2 OK, 3 DATA) and the chip's mode, as `lr20xx_status` decodes
them.

## The status line

Every 20 s while receiving, the log has a line like this:

```
rx 250000 S/s cs16 at 461.33750 MHz: 5000000 samples, 0 losses, 0 restarts, 0 errors, 0 IRQs late; buffer peak
12 KB, 0 link drops, 3 SPI bounces; reader 18.0% of core 0; internal RAM free 120000
```

- **losses:** the chip's FIFO overflowed.
- **link drops:** reads dropped because the host or the network didn't keep up.
- **SPI bounces:** transfers ESP-IDF had to copy through a buffer of its own.

On the W12 the log goes to the USB serial port. With nothing reading the port, log writes can stall.

## Files

| File | What |
|---|---|
| `main/bridge.c` | the chip, the reader, the sample buffer, the link's commands |
| `main/link_tcp.c` | the TCP server and UDP discovery |
| `main/link_ncm.c` | the network over USB (TinyUSB NCM, a DHCP server) |
| `main/link_serial.c` | the serial link, and its USB Serial/JTAG and UART ports |
| `main/link_usb.c` | the serial link's TinyUSB CDC port |
| `main/wifi.c` | Wi-Fi and the serial console |
| `main/http.c` | status and OTA |
| `../lib/hal/esp_idf.c` | the library's HAL on ESP-IDF's SPI master |
