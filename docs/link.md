# The bridge link

How a host drives an LR20xx on a bridge: a microcontroller that has the chip on its SPI bus, reads the FIFO, and
streams the samples (`firmware/`). The encoding is in `proto/include/lr20xx_link.h`; `lr20xx-link` (Rust) is the
same in Rust. Version 1.

## Transports

- **TCP**: the bridge listens on port 7373. One client at a time; a new one replaces the old.
- **USB network** (NCM): on an ESP32-S3, the bridge can also be a USB network adapter. The host gets an address by
  DHCP, and TCP, discovery and HTTP work over it as over Wi-Fi. Each bridge has its own /30 in 10.73.0.0/16.
- **Serial**: the same byte stream over a serial port: the USB Serial/JTAG port that the C3, C6, H2 and S3 have built
  in, a UART, or a TinyUSB CDC port. A UART needs the same baud rate both ends.
- **Discovery**: a host sends the 7 bytes `LR20XX?` by UDP to port 7374 (a broadcast, or one address). Each bridge
  answers the sender with one RSP frame: seq 0, err 0, the HELLO answer.

## Frames

A byte stream of frames, every field little-endian:

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | 0xA5 |
| 1 | 1 | type |
| 2 | 2 | payload length, at most 4096 |
| 4 | n | payload |

| Type | Direction | Payload |
|---|---|---|
| 0x01 REQ | host to bridge | u16 seq, u16 command, the command's arguments |
| 0x02 RSP | bridge to host | u16 seq (the REQ's), i32 error (0 OK, else an `lr20xx_err_t`), the answer |
| 0x10 RX_DATA | bridge to host | u64 first sample's index, i64 bridge time (µs), u8 format, u8 flags, u8 bits, samples (packed) |
| 0x11 TX_DATA | host to bridge | samples for the transmission begun |
| 0x12 EVENT | bridge to host | u16 kind, i32 error, text |

A HELLO starts a session: receiving stops and a transmission begun is dropped. A serial port has no connection to
say a new host is there; this does. Each REQ gets one RSP. A reader that loses sync drops bytes until a 0xA5 starts a header with a known type and a
length in range.

RX_DATA's samples are packed: each value (I, then Q) a `bits`-bit two's complement integer, LSB first, in a
little-endian bit stream, the last byte's spare bits 0. The bridge picks the narrowest width (4 to 16; at most 8 for
int8) that holds the frame's values, so nothing is lost. At full width this is the chip's own format byte for byte:
16 bits is int16 little-endian, 8 is int8. Since a width is at least 4 bits, the padding is under one I/Q pair, and
the count is `floor(bytes × 8 / bits)` rounded down to even. Over full-speed USB this is what fits the fast rates:
int8 is never wider than 6 bits (`docs/iq.md` R2.33), and int16 noise at a moderate gain needs about 10.

RX_DATA's index counts samples delivered since RX_START. Flag 0x01 (GAP) says samples were lost just before this
block: by the chip (a FIFO overflow) or by the bridge (the host didn't keep up). The index doesn't jump at a gap,
so the host can't tell how many were lost; the bridge time can.

## Commands

| Command | Arguments | Answer |
|---|---|---|
| 0x0001 HELLO | | version, ROM, capabilities, the board's range, names (106 bytes) |
| 0x0010 RX_CONFIG | rate (f64), mode, gain, center (f64 Hz), ppm (f64) | the plan: mode, format, rate as num / den, passband |
| 0x0011 RX_START | | (RX_DATA follows) |
| 0x0012 RX_STOP | | (no RX_DATA after the answer) |
| 0x0013 SET_GAIN | i8 step: 0 the AGC, 1 to 13 | |
| 0x0020 TX_BEGIN | mode, center, ppm, rate, fdev, power (f32 dBm), samples (u64), at_us (i64) | (TX_DATA follows) |
| 0x0021 TX_COMMIT | | once the burst has gone out: runs, bytes, TX FIFO flags, early, timed out, key_us (i64), late |
| 0x0022 TX_ABORT | | |
| 0x0030 READ_REG | u32 address, u8 count (1 to 64) | the words |
| 0x0031 WRITE_REG | u32 address, u32 value | |
| 0x0032 RAW | u16 answer bytes, a command frame | the status bytes, the answer |
| 0x0040 STATS | | samples, losses, restarts, errors, reads, IRQs late, link drops, ring peak |
| 0x0041 TEMP | | f32 °C |
| 0x0042 TIME | | i64 the device's clock, us |

READ_REG, WRITE_REG, RAW and TEMP are refused (`LR20XX_ERR_INVALID_STATE`) while receiving. RAW is a debug path:
after it, the chip's state is the host's problem.

A transmission is TX_BEGIN, then exactly `samples` samples in TX_DATA frames (in the mode's format: 2 bytes a sample
for I/Q, 1 for frequency or phase steps), then TX_COMMIT. A bridge whose HELLO has capability 0x08 (TX_STREAM)
also takes `samples` 0: a streamed burst, as long as the TX_DATA that follows, ended by TX_COMMIT. It keys up once
it has enough (0.2 s of samples streamed; for a known length, when the rest will come in before what's in has gone
out, at the link's rate so far) and takes TX_DATA into a ring as the burst goes out: while the ring is full it stops
reading the link, and the link's flow control holds the host. A burst that runs dry holds the last sample (R3.13);
after a second with nothing, the bridge ends it and TX_COMMIT answers `LR20XX_ERR_TIMEOUT`. Without TX_STREAM, the
bridge buffers the whole burst first, and HELLO's `tx_max_samples` is the most it takes. Reception pauses for the
burst and resumes after it, with a gap.

Time: RX_DATA's `t_us`, TIME's answer, `at_us` and `key_us` are one clock, the bridge's since it booted. With
capability 0x10 (TIME), a TX_BEGIN `at_us` other than 0 keys up then: reception goes on until 10 ms before, the FIFO
is filled, and the SetTx goes out at `at_us` (the signal follows by the chip's start-up). Ready more than 100 us past
it (`LR20XX_TX_LATE_US`), nothing is sent: TX_COMMIT answers with `late` 1 and `key_us` 0. `key_us` is when the
first SetTx went out. TX_BEGIN's `at_us` and TX_COMMIT's `key_us` and `late` came with TIME: a shorter TX_BEGIN
(37 bytes) is untimed, a shorter answer (11 bytes) has them 0.
