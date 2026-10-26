# fake_bridge

A stand-in for the bridge firmware: `fake_bridge.py` speaks `proto/include/lr20xx_link.h` over TCP with no chip
behind it, so the SoapySDR module can be run and tested anywhere. Python 3, standard library only.

```sh
tools/fake_bridge/fake_bridge.py --port 7373 --tone 10000 --gap-every 40 --tx-dump /tmp/burst.tx
```

Receiving streams a tone `--tone` Hz off the center in the planned format, paced in real time, and loses half a
frame every `--gap-every` frames (flagged as a gap, the index jumping past it). Its plans only approximate
`lr20xx_iq_plan` (every FSK code is taken as listed). Transmissions are checked for length and written to
`--tx-dump` (the mode byte, then the samples). It answers the UDP discovery on port 7374 on every interface, as
`--serial` (FAKE0001); `--bind` is the TCP port's address only.
