# tools/baselines

PC-coverage baselines extracted from a long reference-emulator run
(SC-55mk2, `../roms/SC-55mk2-v1.01/`, see `../README.md` "Verification workflow"):

- `pc_main.txt` — 6682 unique main-MCU PCs, `XXXXXXXX` per line, sorted
- `pc_sm.txt`   — 188 unique SM sub-CPU PCs, `XXXX` per line, sorted

They are regenerated with `../diff/extract_pc.exe <obs.log> pc_main.txt pc_sm.txt`.
The directory is gitignored except this note, which is tracked so git keeps
the folder in place; drop the two `.txt` files here to make the
verification step runnable.
