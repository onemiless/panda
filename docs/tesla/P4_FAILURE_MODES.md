# P4 pre-change failure modes and validation

Recorded before production edits. Keep lean H7/C4 behavior, restore DOS/F4 only.

- DOS board declaration may introduce unsupported fan_stall_recovery: inspect all board initializers against lean pin.
- F4 flash/RAM overflow or signing incompatibility: build F4 and H7 artifacts, inspect ELF sections against linker script.
- Wrong USB/SPI internal Panda selected: runtime profile matrix, C3XL SPI regression, no device operations here.
- Runtime profile mismatch between Python/C++: compile C++ check and run same explicit profile values/file fallback.
- Boot-chain corruption/wrong board: immutable reference manifests, all six boot partitions field comparison, C4 byte equality, negative allowlist cases.
- Missing profile on tici: shell selection matrix must disable both startup and updater flashing.
- AGNOS boot/system compatibility: offline image inspection; device parent owns lsmod and mutation gates.
- Camera mismatch: bounded on-device checker must reject non-OX03C10 road sensor, dropped frames or low frame rate; no driver port.

Validation artifacts are written to main repository docs/tesla/evidence. Existing upstream tests are preserved, no post-code unit tests will be introduced. Hardware execution/firmware acceptance remain separate gates.
