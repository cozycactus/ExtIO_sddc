# Repository Guidelines

## Project Structure & Module Organization
- `Core/` holds DSP pipelines, radio abstractions, and SIMD variants; keep shared logic here.
- `libsddc/` wraps the core for reuse; `ExtIO_sddc/` provides the Windows ExtIO client; `SoapySDDC/` exposes a SoapySDR module via target `SDDCSupport`.
- `SDDC_FX3/` contains Cypress FX3 firmware sources; supporting vendor utilities live under `SDK/` and board data under `hardware/`.
- `unittest/` hosts CppUnit suites; generated artifacts land in `build/` (create locally, never commit) and helper macros sit in `cmake/`.
- `.github/workflows/cmake.yml` defines the multi-platform CI pipeline; check it when altering packaging or dependencies.

## Build, Test, and Development Commands
- `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release` configures a host build (default toolchain uses C++17 and SIMD flags).
- `cmake --build build --config Release` compiles core libraries, ExtIO plugin, and Soapy module; add `--target unittest` or `--target SDDCSupport` as needed.
- `ctest --test-dir build -C Release --output-on-failure` executes host-side unit tests; rerun after touching DSP or device control paths.
- `make -C SDDC_FX3` rebuilds firmware once `FX3FWROOT`, `ARMGCC_INSTALL_PATH`, and `ARMGCC_VERSION` are exported; commit the refreshed `SDDC_FX3.img` only when firmware changes.

## Coding Style & Naming Conventions
- Follow existing C++ style: tabs for indentation, braces on their own line, PascalCase for classes/methods, camelCase for members, and grouped includes (`"..."` then `<...>`).
- Keep CMake lists two-space indented with lowercase commands; YAML and GitHub Actions files mirror existing spacing and key ordering.
- Name new binaries and sources descriptively (e.g., hardware drivers as `RX888R3Radio.cpp` inside `Core/radio/`).

## Testing Guidelines
- Tests reside in `unittest/*.cpp` and use `TEST_CASE(Fixture, TestName)`; co-locate new fixtures with the component under test.
- Ensure new logic has deterministic coverage; prefer parameterized checks over hardware dependencies and gate SIMD-specific assertions.
- CI runs `ctest` in Release mode; add local debug checks (`USE_DEBUG_ASAN=ON`) when investigating memory issues.
- Document any manual hardware verification in the PR when unit coverage is impractical.

## Commit & Pull Request Guidelines
- Write concise, imperative commits (e.g., `Fix build in MSYS2 MinGW64`); avoid leaving `Save wip` once feedback is addressed.
- Rebase before opening PRs, link related issues, and separate host, firmware, and packaging notes within the description.
- Include evidence of local runs: `ctest` output, firmware build logs, or `SoapySDRUtil --info` when touching the module.
- CI spans Windows, Linux, and macOS; confirm platform-specific options (`USE_SIMD_OPTIMIZATIONS`, `USE_DEBUG_ASAN`) before requesting review.

## Firmware & Hardware Notes
- Update `hardware/` tables when adding boards; keep configuration comments concise and hardware-specific.
- Do not check in release archives (`SDDC_EXTIO.ZIP`, `SDDC_SOAPY.ZIP`); rely on the GitHub Actions artifacts defined in `.github/workflows/cmake.yml`.
