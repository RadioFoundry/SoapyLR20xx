# lib

`liblr20xx`: a Semtech LR20xx on SPI, and the chip as an SDR. Portable C11. Written from the datasheet (DS.LR20xx
Rev. 2.2) and, for the undocumented raw paths, from [docs/iq.md](../docs/iq.md) (cited as R*n*).

| File | What |
|---|---|
| `include/lr20xx_cmd.h`, `lr20xx_radio.h`, `lr20xx_lora.h`, `lr20xx_fsk.h`, `lr20xx_proto.h` | every command's frame and answer: no I/O |
| `include/lr20xx_tx.h` | the transmit modes' setups and sample formats: no I/O |
| `include/lr20xx.h`, `src/lr20xx.c` | the SPI driver: reset, BUSY, command and answer frames, the FIFOs; transmit streaming and packets |
| `include/lr20xx_hal.h` | what the driver needs of the pins; `hal/linux.c` (spidev and GPIO lines), `hal/esp_idf.c` |
| `include/lr20xx_port.h` | errors, a clock and waits; `port/posix.c`, `port/esp_idf.c` |
| `include/lr20xx_iq.h`, `src/lr20xx_iq.c`, `src/lr20xx_iq_plan.c` | the raw I/Q session: modes and rates, the reader, transmit bursts |
| `include/lr20xx_board.h` | the board around the chip: clock, RF switch, IRQ DIO, range |
| `include/lr20xx_chips.h` | the ROMs the raw paths are known on |
| `test/` | host tests: the encodings byte for byte, the driver over a mock HAL, the session over a model of the chip |

Most of it came from RadioFoundry Gateway (`tools/upstream/`); keep those files close to upstream.

## Use

```c
lr20xx_linux_cfg_t lc = {.spidev = "/dev/spidev0.0", .gpiochip = "/dev/gpiochip0", .busy = 24, .nreset = 25,
                         .irq = -1, .pa_en = -1};
lr20xx_config_t c = {0};
char err[96];
lr20xx_hal_linux_open(&lc, &c.hal, err, sizeof err);
lr20xx_t *dev;
lr20xx_open(&c, &dev);
static lr20xx_iq_t iq;
lr20xx_iq_init(&iq, dev, &lr20xx_board_generic, false);
lr20xx_iq_cfg_t cfg = {.rate = 250000, .center_hz = 915e6, .gain = 13};
lr20xx_iq_start(&iq, &cfg);
for (;;)
    lr20xx_iq_poll(&iq, sink, NULL, 5); /* on a thread of its own, at a real-time priority */
```

The FIFO lasts 2 ms. Run the reader on its own thread at a priority nothing else preempts; everything else that
touches the chip goes between its polls.
