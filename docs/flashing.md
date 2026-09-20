# Flashing

The firmware is in [firmware/sentry_node](../firmware/sentry_node/). Building it needs two things: ESP-IDF v6.0 with the ESP32-S3 toolchain (tested with v6.0.2), and a Python environment with numpy, which the build uses to generate the detector's constants, tables and test vectors from [src/](../src/) and [data/](../data/).

## Setting up

Install ESP-IDF v6.0 by following Espressif's getting started guide, and open a terminal where `idf.py` works. The Python side needs numpy and nothing else, and the build looks for an interpreter itself: it takes the first one it finds that can actually import numpy, starting with the environment `environment.yml` describes and ending with whatever is on PATH. If you already have a Python 3 with numpy, there is nothing to set up.

If you do not, either install numpy where you want it:

    python -m pip install numpy      # py -m pip install numpy on Windows

or create the development environment from the root of this repository, which pins the versions the detector was measured with:

    conda env create -f environment.yml

conda is not required. Any Python 3 with numpy writes the same headers, byte for byte. To name an interpreter yourself instead of letting the build look:

    export SENTRY_PYTHON=/path/to/python            # macOS, Linux
    $env:SENTRY_PYTHON = "C:\path\to\python.exe"    # Windows PowerShell
    set SENTRY_PYTHON=C:\path\to\python.exe         # Windows cmd

`idf.py -DSENTRY_PYTHON=<path> build` does the same thing. When no interpreter is found the build stops while it is configuring and names the ones it tried, instead of failing part way through.

## Building and flashing

The breadboard build, on an ESP32-S3-DevKitC-1, is the default target:

    cd firmware/sentry_node
    idf.py build
    idf.py -p PORT erase-flash flash monitor

The full unit, on PCB rev A2, builds into its own directory with its own configuration:

    cd firmware/sentry_node
    idf.py -B build_pcb_a2 -DSDKCONFIG=build_pcb_a2/sdkconfig -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.pcb_a2" build
    idf.py -B build_pcb_a2 -p PORT erase-flash flash monitor

Keep the two builds in separate directories. The `-DSDKCONFIG` option is what stops the PCB configuration leaking into the breadboard build. The target changes the pin map and the hardware the board has, such as the battery monitor and the LoRa radio, and nothing else. The detector is the same code on both.

Three things that will save you an evening.

1. **Never assume the serial port.** The ESP32-S3's native USB re-enumerates on every reset and the name differs between bootloader and application. List ports from the terminal each time, and if `idf.py` picks the wrong one give it `-p`.
2. **Erase before the first flash of a new unit.** The image expects clean settings storage. Leftovers from an interrupted flash produce confusing boots. Later flashes of the same unit can leave out `erase-flash`.
3. **Use a data cable.** Charge-only USB-C cables are the most common reason a board does not show up, ahead of every real fault seen so far.

The unit boots to LISTENING in about two seconds and needs nothing else. The serial console stays available. Send `I` for the pin map and the board the image was built for, and watch the frame line for the score, the best rate and the tier states.
