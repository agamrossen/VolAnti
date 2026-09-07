# Flashing

Requires ESP-IDF v6 with the ESP32-S3 toolchain installed.

    cd firmware/volanti
    idf.py set-target esp32s3
    idf.py menuconfig          # only to change the board target. Default is the PCB.
    idf.py erase-flash flash monitor

For the breadboard build select the `devkit` board target before building. The target changes the pin map and nothing else. The detector is the same code.

Three things that will save you an evening.

1. **Never assume the serial port.** The ESP32-S3's native USB re-enumerates on every reset and the name differs between bootloader and application. List ports from the terminal each time, and if `idf.py` picks the wrong one give it `-p`.
2. **Erase before the first flash of a new unit.** The image expects clean settings storage. Leftovers from an interrupted flash produce confusing boots.
3. **Use a data cable.** Charge-only USB-C cables are the most common reason a board does not show up, ahead of every real fault seen so far.

The unit boots to LISTENING in about two seconds and needs nothing else. The serial console stays available. Send `I` for the pin map, and watch the frame line for the score, the best rate and the tier states.

The firmware source lands in [firmware/](../firmware/) with the first tagged release.
