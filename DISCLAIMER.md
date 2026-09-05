# Disclaimer

Read this before building, deploying, or relying on VolAnti.

## What this is

VolAnti is an open-source, community-built acoustic early-warning aid. It was designed and tested by a student engineering project. It is published so that people who need something like it can build, inspect, and improve it.

## What this is not

- **It is not a certified life-safety system.** No part of it has been tested or certified to any safety standard (functional safety, alarm system, or otherwise).
- **It does not guarantee detection.** Wind above roughly 5 m/s sharply reduces range. Quiet-by-design aircraft, masking noise, terrain, and device faults can all cause a drone to arrive with no warning. Zero false alarms in our testing does not mean zero missed detections in yours.
- **It is not a substitute for shelter, official warning systems, or safe behaviour.** Treat every alert as a prompt to act, and treat silence as absence of information, never as confirmation of safety.
- **It cannot distinguish friend from foe.** Any multirotor in range can trigger it, including your own.

## Your responsibilities if you build or deploy it

1. **Test your build before depending on it.** At minimum, run the bench verification in [docs/build-guide.md](docs/build-guide.md) and a live rotor test at a known distance in your actual deployment conditions.
2. **Comply with local radio regulations.** The LoRa alert frequency is a configuration parameter, not a default you can ignore. 868 MHz is legal in the EU/UK; other regions differ (Israel allocates 917–920 MHz; the US uses 915 MHz). Verify your local allocation before transmitting. See [docs/configuration.md](docs/configuration.md).
3. **Comply with local law generally**, including any rules on acoustic recording in your jurisdiction. The device processes audio in real time and stores spectral scores and event metadata, not recordings, but you are responsible for how you use it.
4. **Battery safety is yours.** Use only protected 1S lithium-polymer cells, verify connector polarity with a meter before first connection, inspect for swelling on the schedule in the deployment guide, and never charge unattended in the first hours of a new build.

## Scope boundary

This project is detection and peer alerting only. Contributions that add jamming, countermeasures, direction-finding for targeting, or any transmit capability beyond the alert packet will not be accepted, in any form, for any stated purpose.

## Liability

The hardware designs, firmware, and documentation are provided "as is", without warranty of any kind, express or implied, including fitness for a particular purpose. To the maximum extent permitted by law, the authors and contributors accept no liability for any loss, injury, or damage arising from building, deploying, or relying on this project. See the license texts in [LICENSES/](LICENSES/) for the governing terms.
