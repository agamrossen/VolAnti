# Flashing the firmware

Two routes. The browser route needs nothing installed; the toolchain route is for anyone who wants to modify the code.

## Route 1: browser (recommended for builders)

<!-- ESP-WEB-TOOLS: enable this block when the first tagged release publishes binaries. -->
Open the web flasher *(link lands with the first release)* in Chrome or Edge, plug the unit in over USB-C, click **Install**, pick the port, choose your board (`VolAnti PCB` or `DevKit breadboard`), and wait ~90 seconds. Done. The flasher uses ESP Web Tools; it erases the flash and writes the complete validated image, so the unit powers up straight into guarding with zero configuration.

## Route 2: ESP-IDF toolchain (for developers)

Requires ESP-IDF v5.x with the ESP32-S3 toolchain installed.

```
cd firmware/volanti
idf.py set-target esp32s3
idf.py menuconfig        # only if changing the board target; defaults = production PCB
idf.py erase-flash flash monitor
```

For the breadboard build, select the `devkit` board target before building. The board identity is compiled into the binary and read back by the release certificate, so a binary always knows which hardware it is for.

## Three things that will save you an evening

1. **Never hardcode or assume the serial port.** The ESP32-S3's native USB re-enumerates on every reset and differs between bootloader and application mode. List ports fresh each time (`ls /dev/cu.usb*` on macOS, `ls /dev/ttyACM*` on Linux) and confirm from the terminal before flashing.
2. **First flash of a new unit: always `erase-flash` first.** The shipped image expects clean settings storage; stale partial settings from an interrupted flash produce confusing half-configured behaviour.
3. **Use a data-rated USB-C cable.** Charge-only cables are the most common cause of "the board doesn't show up", ahead of every actual hardware fault we have seen.

## After flashing

The unit boots to guarding in about 2 seconds, runs its output parade once (chirp, LED, display refresh), and needs nothing else. The serial console remains available for the checks in the build guides (`I` prints the pin map, per-channel mic health prints at boot, `V` dumps the event ring).
