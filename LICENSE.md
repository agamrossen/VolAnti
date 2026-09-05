# Licensing

VolAnti uses three licenses, one per domain. This is deliberate: a single license fits none of the three kinds of work in this repository well.

| Domain | Paths | License | Why |
|---|---|---|---|
| **Hardware** | `hardware/` | [CERN-OHL-W-2.0](LICENSES/CERN-OHL-W-2.0.txt) | The weakly-reciprocal open-hardware license. Modified board or enclosure designs must be shared back, but products that merely *incorporate* the design are not captured. This keeps derivatives open without blocking someone from, say, integrating the array into a larger system. |
| **Firmware & software** | `firmware/`, `tools/` | [Apache-2.0](LICENSES/Apache-2.0.txt) | Permissive with an explicit patent grant. Maximum reuse of the detector algorithms, and the patent clause protects builders. |
| **Documentation & media** | `docs/`, `*.md`, images | [CC-BY-SA-4.0](LICENSES/CC-BY-SA-4.0.txt) | Translations and adapted guides must stay open and credit the source. Translation is one of the highest-value contributions this project can receive. |

## Practical answers

- **Can I sell assembled units?** Yes, under all three licenses. If you modify the hardware design, publish your changes (CERN-OHL-W). You may not imply certification or endorsement that does not exist, and you carry the [DISCLAIMER](DISCLAIMER.md) obligations to your customers.
- **Can I use the detector code in a closed product?** Yes (Apache-2.0). Attribution required.
- **Can I translate the docs?** Please do. CC-BY-SA: credit the project, share alike.
- **Trademark:** "VolAnti" is the name of this project. Use it to refer to unmodified builds; call forks something else so build reports stay meaningful.

OSHWA self-certification (open-source hardware certification mark) is planned once the repository reaches its first tagged release.
