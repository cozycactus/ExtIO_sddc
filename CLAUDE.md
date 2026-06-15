# ExtIO_sddc — notes for Claude Code

Fork: `cozycactus/ExtIO_sddc` (origin). Upstream: `ik1xpv/ExtIO_sddc`.
Working branch: `dev`.

This repo builds the SDDC stack for RX888 / BBRF103 / HF103 (Cypress FX3 based SDRs):
- `Core/` — shared DSP + USB/firmware handling (`SDDC_CORE` static lib).
- `libsddc/` — C shared library `libsddc` + CLI test tools (`sddc_test`, `sddc_stream_test`).
- `SoapySDDC/` — SoapySDR plugin `libSDDCSupport.so` (factory name **`SDDC`**).
- `ExtIO_sddc/` — Windows ExtIO DLL (MSVC only).

## Build (macOS — Apple Silicon and Hackintosh)

Deps via Homebrew:
```bash
brew install cmake pkg-config libusb fftw soapysdr
```
Configure + build + install the SoapySDR module:
```bash
rm -rf build                                   # see gotcha below
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j8
sudo cmake --install build                     # installs libSDDCSupport.so into SoapySDR modules
```

### Gotcha: stale CMakeCache after a Homebrew upgrade
Root `CMakeLists.txt` resolves fftw/libusb via `pkg_check_modules`, and the
*versioned* Cellar include path (e.g. `.../fftw/3.3.10_2/include`) gets cached in
`build/CMakeCache.txt`. When Homebrew bumps the keg revision (`_2` -> `_3`) that
directory disappears and the build fails with `fatal error: 'fftw3.h' file not
found`. A plain reconfigure does NOT fix it (pkg-config results are cached).
**Always `rm -rf build` and reconfigure** after a brew upgrade. `build/` is gitignored.

## Verify the SoapySDR plugin
```bash
SoapySDRUtil --info                  # should list module libSDDCSupport.so, factory SDDC
SoapySDRUtil --find="driver=SDDC"    # enumerates; bootloader shows as "WestBridge"
SoapySDRUtil --probe="driver=SDDC"   # opens -> uploads FX3 firmware -> prints capabilities
```

## Hardware / USB facts
- FX3 USB IDs: `04b4:00f3` = bootloader (un-programmed), `04b4:00f1` = streamer
  (after firmware upload). VID `0x04b4` = Cypress; bootloader product string is
  "WestBridge".
- `--find` just enumerates. Opening the device (`makeSDDC` / `--probe`) uploads the
  embedded firmware in a single process: find bootloader -> `load_image` -> wait
  ~500 ms -> re-find as streamer (`Core/arch/linux/usb_device.c:usb_device_open`).
- The open path **requires SuperSpeed or High-Speed**; a Low/Full-speed link is
  rejected ("USB 3.x SuperSpeed ... connection failed"). So the device must sit on
  a real USB3 port.
- **RX888 (original) is bus-powered only** and draws spiky current. On weak ports,
  cheap USB-C adapters, or behind hubs it **browns out**: enumerates as bootloader
  for ~1 s, then drops off the bus before it can be opened. Fix is a solid USB3
  cable into a direct, well-powered port (rear motherboard port / powered hub).

### Hackintosh-specific
Make sure the USB port is correctly mapped as **USB3 / SuperSpeed** in the USB
port-injection kext (USBMap / USBToolBox). An unmapped/mismapped port can present
at reduced speed/power and trigger the same brown-out / speed-rejection.

## Current status (2026-06-16)
- libsddc + SoapySDDC build and load cleanly on macOS Apple Silicon; `SDDC` factory
  registers and runs (`--find`/`--probe` work without hardware → "No devices found").
- RX888 was detected on a MacBook (label `SDDC :: WestBridge sn:0000000004BE`) but
  **browned out** (~1.3 s on bus, then dropped) through a USB-C/hub path → could not
  be probed. Plan: move to Hackintosh Tahoe with a direct rear USB3 port.
- Added a null/return-value check in the `SoapySDDC` constructor
  (`SoapySDDC/Settings.cpp`): a failed `Fx3->Enumerate()`/`Fx3->Open()` now throws a
  clean `std::runtime_error` instead of dereferencing a null device and
  **segfaulting**. (Previously `--probe` crashed when the board dropped mid-open.)

### Next steps
1. On Hackintosh: install deps, `rm -rf build`, rebuild, `sudo cmake --install build`.
2. Plug RX888 into a direct rear USB3 port (good cable); confirm it stays enumerated
   in bootloader (`04b4:00f3`) without flapping.
3. `SoapySDRUtil --probe="driver=SDDC"` → firmware upload + full device readout.
4. Then drive it via SoapySDR (e.g. CubicSDR / SoapySDR apps).
