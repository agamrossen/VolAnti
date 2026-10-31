# Security and privacy notes

What the unit captures, what leaves it, and what an attacker within radio
range can do. Written from the code as of the rev A2 / Tier-4 build; file
references point at the source so each claim can be checked.

## Data the unit captures

Four ICS-43434 microphones at 16 kHz feed the detector (`source_i2s_quad.c`).
Audio exists in RAM only:

- No flash, SD or NVS writes anywhere in the detector path. The only
  persistent store is `settings.c`, which keeps detection thresholds and
  output configuration in NVS (`settings.c:93-131`).
- The event log is a 64-entry RAM ring of alert decisions, stamped in uptime
  milliseconds, lost on power removal (`event_log.h`).
- Raw PCM leaves the device only over the USB console when a host explicitly
  runs the `parity` or `X` capture modes (`sentry_node.c` `run_parity`,
  TRACE_MAGIC_PCM). The standalone field loop never streams samples.
- What goes on air is one 18-byte packet: magic, version, flags, a 16-bit
  device ID, sequence, hop, tier, epoch, score and threshold, then a CRC-16
  (`lora_proto.c`, `lora_proto.h`). No audio, no location, no timestamp.

## Radio: unauthenticated by design (v0)

The LoRa link has no encryption and no message authentication; the only
integrity check is CRC-16 (`lora_proto.c:lora_crc16`). Consequences, so that
users can weigh them deliberately rather than discover them:

- **Spoofed alerts.** Anyone in range on the same frequency can transmit a
  valid 18-byte packet and trigger the buzzer, motor and LED on paired units.
  CRC-16 is an error check, not a credential. The parser is length-, magic-,
  CRC-, version- and hop-gated (`lora_pkt_parse`), which stops malformed
  frames and relay storms, not forgery.
- **Presence tracking.** `device_id` is CRC-16 of the factory eFuse MAC and is
  constant for the life of the unit (`lora_link.c`, `build_frame`). A passive
  listener can distinguish and follow individual units by that ID.
- **Probe-driven disclosure.** A LINKREQ frame is answered with a LINKACK
  burst, so a probing third party can confirm a unit is powered and in range
  (`lora_link.c:343-350`).
- **Replay.** Deduplication holds 16 keys for 60 s (`lora_proto.h`
  LORA_DEDUP_*). A captured burst replayed after that window is accepted as a
  fresh alert.

Mitigations for a deployment that needs them, without forking the protocol:
separate channel/SF from any other users, physical separation of paired units,
and treating the buzzer as advisory rather than as a trigger for any action
that matters. An authenticated packet (keyed MAC over bytes 0-15, key
provisioned per pair) is the v1 fix and would need a version bump.

## Physical / console access

The USB console is unauthenticated and always available
(`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`); anyone with the USB cable can read
status, change thresholds, enable tiers 3/4, or force alerts. There is no
flash encryption or secure boot. Expected for an open dev board; assume that
a person with the device owns the device.

## Network

The firmware contains no Wi-Fi, Bluetooth, HTTP, MQTT, DNS or OTA code. The
only ESP-IDF network-adjacent call is `esp_read_mac()` for the device ID
(`lora_link.c`). The Python tooling talks to local serial ports only. The
website is a static page whose only third-party requests are Google Fonts.

## Watchdogs

`CONFIG_ESP_TASK_WDT_INIT=n` and `CONFIG_ESP_INT_WDT=n` are set in
`sdkconfig.defaults` for replay determinism and therefore apply to the field
build too: a hung detector is silent rather than reset. The brownout detector
stays at IDF defaults. If you deploy units unattended, consider enabling the
task watchdog and re-checking the golden gate.

## Reporting

Open a GitHub issue for anything concrete; include firmware version, board
and a capture if the radio is involved.
