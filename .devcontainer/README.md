# Dev container (optional)

A reproducible Linux environment to **build, unit-test and edit** this STM32WL55
firmware without setting up the toolchain by hand.

## Use it
- **VS Code**: install the *Dev Containers* extension, open the repo, then
  *"Reopen in Container"*. First build pulls Ubuntu + the ARM toolchain (~a few
  minutes, one time).
- **CLI**: `devcontainer up --workspace-folder .` (devcontainers/cli), or build
  the image directly: `docker build -t argos-fw .devcontainer`.

## What's inside
- `arm-none-eabi` **GCC 13.3.rel1** — pinned to match the validated host
  toolchain (same `-Werror` behaviour).
- native `gcc` + `make` for the Unity unit tests.
- `clangd` + `bear` for IDE intellisense; `postCreateCommand` generates
  `compile_commands.json` on first start.
- `python3`, `gdb-multiarch`, `minicom`, `stlink-tools`.

## Build / test inside the container
```sh
make help                       # canonical build lines
make BOARD=SMD_STDALONE APP=UW_DOPPLER COMM=UART DEBUG=0 MAC_PRFL=BASIC REED_WKUP3_WIRE=1 -j full
cd Tests && bash scripts/run_tests.sh   # unit tests (run cleanly here; /tmp is writable)
```

## Flashing
`make flash*` drives a **SEGGER J-Link on the host** (proprietary, not bundled).
USB passthrough into a container is unreliable on Windows/macOS, so **flash from
the host**. On a Linux host with an ST-LINK probe you can use the bundled
`st-flash` against the built `build/*.bin` (the `--privileged` + `/dev` mount in
`devcontainer.json` exposes the USB device).
