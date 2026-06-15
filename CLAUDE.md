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
- **Now also built + installed on the Hackintosh** (MacPro7,1, x86_64, Darwin 25.5.0):
  deps present (cmake 4.3.3, pkgconf 2.5.1, libusb 1.0.30, fftw 3.3.11, soapysdr
  0.8.1), `rm -rf build` + reconfigure + `cmake --build -j8` clean (warnings only).
  Installed module is now `libSDDCSupport.so (1.0.1-3d602ce)` in
  `/usr/local/lib/SoapySDR/modules0.8/` — replaced a stale `1.0.1-122ea61` build that
  predated the segfault fix. NOTE: `/usr/local/lib/SoapySDR/modules0.8` is owned by
  `cozy:admin` here, so `cmake --install build` works **without sudo**.
- RX888 was detected on a MacBook (label `SDDC :: WestBridge sn:0000000004BE`) but
  **browned out** (~1.3 s on bus, then dropped) through a USB-C/hub path → could not
  be probed. Moved to Hackintosh Tahoe with a direct rear USB3 port (this machine).
  As of this rebuild the RX888 is **not yet plugged in** (`--find` → "No devices
  found", no Cypress FX3 on the bus) → hardware probe still pending.
- Added a null/return-value check in the `SoapySDDC` constructor
  (`SoapySDDC/Settings.cpp`): a failed `Fx3->Enumerate()`/`Fx3->Open()` now throws a
  clean `std::runtime_error` instead of dereferencing a null device and
  **segfaulting**. (Previously `--probe` crashed when the board dropped mid-open.)

### Hardware bring-up result (2026-06-16, Hackintosh)
- RX888 plugged into a rear USB3 port: enumerates as bootloader and **stays stable**
  (5/5 finds over 5 s, no brown-out flapping — the MacBook failure mode is gone).
  `--find` → `SDDC :: WestBridge sn:0000000004BE`.
- `SoapySDRUtil --probe="driver=SDDC"` **fully succeeds**: uploads FX3 firmware
  ("writing image... transfer execution to Program Entry"), re-enumerates as
  streamer (`SDDC :: RX888 sn:0009023100C6142F`), prints full readout —
  hardware=RX888, 1 Rx, antennas HF/VHF, freq 0.01–1800 MHz, gain 0–65.5 dB,
  sample rates 2/4/8/16/32/64 MSps, settings biastee/dithering/randomization.
  **This was the original blocker and it is now cleared.**

### Streaming VERIFIED via SoapySDR (the real path) — device fully working
- Wrote a minimal SoapySDR CF32 smoke-test (`/tmp/soapy_sddc_stream.cpp`, build with
  `c++ -std=c++17 -O2 ... $(pkg-config --cflags --libs SoapySDR)`): opens
  `driver=SDDC`, 2 MSps @ 10 MHz, 1.5 s. Result: **opened RX888, 2,998,272 samples in
  1.5 s (~exactly 2 MSps), every sample non-zero → IQ DATA FLOWING ✓.** Low magnitude
  (peak ~2e-4) is expected with no antenna. So find → firmware upload → live IQ
  streaming all work on the Hackintosh. CubicSDR / SoapySDR apps will work.

### FIXED (2026-06-16): `libsddc` CLI streaming now works — CF32 I/Q (default) + raw int16
Previously `sddc_stream_test` / `sddc_vhf_stream_test` printed "started streaming ..."
then **hung forever with no data**. Root cause was stub code in libsddc, not hardware:
`sddc_open` passed `new rawdata()` — a no-op `r2iqControlClass` that started no worker
thread, so nothing drained `inputbuffer` → ringbuffer fills (QUEUE_SIZE=32) →
`fx3handler::PacketRead` blocks in `WaitUntilNotFull()` → deadlock. Confirmed via
`sample <pid>` (FX3 poll thread blocked on a full inputbuffer, `OnDataPacket` blocked on
an empty outputbuffer, zero `r2iqThreadf` threads). The bridge `Callback` was also empty.

**Output model (two selectable formats; default = I/Q).** The RX888's LTC2208 is a real
16-bit ADC that does NOT output I/Q — the I/Q is produced by the on-host `fft_mt_r2iq`
("real-to-IQ") software digital down-converter. Since the SDDC *is* that software DDC, its
natural product is tuned complex I/Q, so that is libsddc's **default**. Raw real int16
(the un-downconverted ADC stream, e.g. for full-band capture) is available as an option.
Select with `sddc_set_stream_format()` (call it BEFORE `sddc_set_sample_rate`, since the
rate meaning differs):
- `SDDC_STREAM_CF32` (default): interleaved float32 I/Q from the DDC; `data_size` bytes,
  complex count = `data_size/(2*sizeof(float))`. `sddc_set_sample_rate` = decimated I/Q
  output rate (idx 0..5 = 2/4/8/16/32/64 MSps for a 128 MHz ADC).
- `SDDC_STREAM_INT16`: raw real int16 ADC samples; count = `data_size/sizeof(int16_t)`.
  `sddc_set_sample_rate` = ADC clock (clamped to [8 MHz, 128 MHz]).

What was done (`libsddc/libsddc.cpp`):
- New controller `libsddc_output : public fft_mt_r2iq`. Default (CF32): inherits and runs
  the real DDC; the bridge `Callback` forwards the processed buffer as CF32. Raw mode:
  `TurnOn()` instead starts a worker draining `inputbuffer` → int16 to the user callback;
  `TurnOff()` stops both ring buffers (releasing the worker and `OnDataPacket`) and joins.
  Inheriting (not embedding) means decimate/sideband/tune (`setFreqOffset`) all work for
  free in I/Q mode. `sddc_open` installs `new libsddc_output(handle)` (handle = context).
- `sddc_set_stream_format()` (new) + format-aware `sddc_set_sample_rate()` (see above).
- `sddc_handle_events` sleeps ~1 ms (was a no-op) so a tight polling loop doesn't peg a CPU.
- `libsddc/CMakeLists.txt`: `sddc` target now adds `${LIBFFTW_INCLUDE_DIRS}` (libsddc.cpp
  includes `fft_mt_r2iq.h` → `fftw3.h`).
- `sddc_stream_test.c` / `sddc_vhf_stream_test.c`: optional 5th arg `iq|raw` (default iq);
  interpret CF32 vs int16 and write a raw `.cf32` (numpy `complex64`) or 16-bit PCM WAV
  accordingly. Kept the **internal wall-clock watchdog** (~2× runtime + 1 s).
- `Core/arch/linux/FX3handler.cpp`: initialize `devidx = 0` in the ctor (win32 ctor already
  did). `sddc_open` never calls `Enumerate()`, which left `devidx` uninitialized when
  `usb_device_open()` selected the device — latent UB, now fixed.

Verified on real RX888 (this Hackintosh), both modes, clean start/stop, no hang:
- CF32 `... 2000000 1000 out.cf32 iq` → 1,998,848 complex IQ samples, ~1969 kSps (2 MHz
  decimated output), file = samples×8 B, 100% non-zero.
- RAW `... 16000000 1000 out.wav raw` → 15,990,784 int16 samples, ~15,944 kSps (16 MHz ADC
  clock), valid 16-bit PCM WAV, real ADC noise (100% non-zero, min/max 125/211, std ~8.2,
  narrow band ⇒ ADC randomization off). No antenna → low levels, as expected.

- Harness gotcha unchanged: do NOT run `SoapySDRUtil --find` (a second opener) concurrently
  with a streaming session — it contends for the device and can hang the open. The
  wall-clock watchdog bounds the wait loop, but an external `timeout` is still good
  belt-and-suspenders.

### Next steps
1. ~~On Hackintosh: install deps, `rm -rf build`, rebuild, install module.~~ **Done.**
2. ~~Confirm RX888 enumerates stable in bootloader.~~ **Done.**
3. ~~`SoapySDRUtil --probe` → firmware upload + readout.~~ **Done.**
4. ~~Verify live IQ streaming.~~ **Done via SoapySDR** (see above).
5. Drive it from a real app (CubicSDR / GQRX-via-Soapy) with an antenna.
6. ~~Fix the `libsddc` streaming stub so the C CLI tools work.~~ **Done** (see FIXED above).
7. ~~Fix libsddc's sample-rate mapping (requested 2 MSps → actual ~32 MSps).~~ **Done**
   (see FIXED — sample-rate mapping above).
