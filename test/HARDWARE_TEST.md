# Passive UART flight test

This build has `HAB_SERIAL_PACKET_TEST_MODE=1` in `src/mission_config.h`.
It runs the normal mission logic and ignores serial input. WSPR and LoRaWAN RF
remain controlled by their independent authorization gates.

On boot, storage is mounted non-destructively three times. If every attempt
fails, firmware formats and remounts LittleFS automatically, clears stale queue
membership/capture time, and records a storage repair. This makes a blank first
boot self-initializing, but a format necessarily erases previously stored images.

The temporary observer adds only two outputs:

- `HAB_STATUS` reports GPS/time freshness, position, geofence, network state,
  image-transfer state, queue depth, and status flags every five seconds.
- After the normal GPS/time-gated mission scheduler stores an image, a separate
  encoder copies its packets to UART at one packet per second. This shadow copy
  does not advance or delete the mission queue. Packets from real LoRaWAN send
  attempts are also mirrored and the receiver deduplicates them.

Connect UART ground and ESP32 UART0 TX. UART0 RX is not required for this test.
Close PlatformIO's serial monitor because only one program can own a COM port.

Install the PC dependencies:

```powershell
py -3 -m pip install -r tools\requirements-hardware-test.txt
```

Start the passive receiver before powering or resetting the board:

```powershell
py -3 tools\hardware_packet_test.py --port COM3 --output hardware-test-output
```

UART0 starts with a five-second quiet delay before the first boot message or
mission initialization, giving the monitor time to attach after a reset.

The receiver sends nothing. It displays ordinary firmware diagnostics and
flight status, verifies the CRC on each `HAB_PACKET`, and saves packets as they
arrive. It can be stopped and restarted; valid packets in `packets.jsonl` are
loaded again automatically.

Metadata packets retain the 210-byte wire format. Data packets are variable
length and end immediately after the encoded JPEG scan bytes; the UART CRC and
the LoRaWAN application-payload length cover only the bytes actually sent.

Normal capture conditions still apply. The board needs fresh GPS position and
UTC time, enough LittleFS space, and a UTC minute in `0-1`, `6-7`, `12-13`, and
so on. If the mission queue has no previous capture timestamp, the first
eligible window captures immediately. Subsequent images retain the normal
eight-hour interval.

When all data packets for an image have arrived, the receiver automatically
creates:

- `image_<id>/reconstructed.png`
- `image_<id>/coverage.png`
- `image_<id>/capture.json`

A green coverage map, recognizable reconstructed image, and `IMAGE <id> PASS`
validate the normal camera capture, PSRAM, LittleFS mission save, telemetry,
queue encoder, packet pacing, UART transport, CRC, packet ordering, MCU
coverage, and independent JPEG entropy decoding.

This test does not alter capture timing or simulate network coverage. With the
current RF gates off there will be no over-the-air transmission; the shadow
copy still lets the complete stored-image packet path be observed. It does not
prove RF power, antenna matching, frequency accuracy, LoRaWAN delivery, or WSPR
reception.

After bench testing, restore `HAB_SERIAL_PACKET_TEST_MODE` to `0` and rebuild.
