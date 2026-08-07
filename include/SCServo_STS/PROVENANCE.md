# Provenance — Changes from Upstream

This directory is a trimmed, STS-only vendored copy of
[adityakamath/SCServo_Linux](https://github.com/adityakamath/SCServo_Linux),
embedded directly in `sts_hardware_interface` (not a git submodule). It
exists purely to build the `SCServo` static library that
`sts_hardware_interface` links against for Feetech STS-series servo control. This SDK also supports SMS-series servos, which are protocol-identical to STS-series, so the `SMS_STS` class is used for both families. However SMS-series servos haven't been tested in this project.

For the full multi-protocol SDK (SMS, SCSCL, HLSCL, SCS0009 support),
documentation, and example programs, see the upstream repository:

- Upstream README: https://github.com/adityakamath/SCServo_Linux/blob/main/README.md
- Upstream docs: https://github.com/adityakamath/SCServo_Linux/tree/main/docs

## What changed from upstream

### 1. Not a git submodule

Vendored as plain source files instead of a git submodule, so
`sts_hardware_interface` builds without needing `git submodule update --init`.

### 2. Non-STS servo families removed

Upstream supports four servo protocol families (SMS/STS, SCSCL, HLSCL,
SCS0009) through a shared `SCS`/`SCSerial` base and separate application-layer
classes. Only the STS-related code is needed here, so the rest was removed:

- **Removed headers:** `HLSCL.h`, `SCSCL.h`, `SCS0009.h`
- **Removed sources:** `HLSCL.cpp`, `SCSCL.cpp`, `SCS0009.cpp`
- **Kept:** `SCS`/`SCSerial` (shared base), `SMS_STS` (the STS application-layer
  class — see note below on naming), and shared utilities (`INST.h`,
  `ServoErrors.h`, `ServoUtils.h`, `SyncWriteBuffer.h`)
- `SCServo.h` (the master include) now only pulls in `SMS_STS.h`
- `CMakeLists.txt` source/header lists trimmed to match
- Doc comments in shared headers (`INST.h`, `SCS.h`, `SCSerial.h`,
  `ServoUtils.h`, `SyncWriteBuffer.h`) that referenced the removed
  `SCSCL`/`HLSCL` classes were rewritten, not just stripped, so the
  surrounding sentences still read correctly

### 3. Class kept as `SMS_STS` (not renamed to `STS`)

Upstream's `SMS_STS` class and `SMS_STS_*` register macros cover both SMS and
STS series servos identically — same protocol, no separate code paths, the
name just historically covers both. An earlier pass in this fork renamed the
class to `STS`, but since there's no actual SMS/STS code split to trim, that
rename was reverted: the class, file names (`SMS_STS.h`/`SMS_STS.cpp`), include
guard, and all `SMS_STS_*` register macros match upstream exactly, unmodified.

`sts_hardware_interface`'s own hardware interface code
(`sts_hardware_interface.hpp`/`.cpp`) uses this class as-is: `#include
<scservo/SMS_STS.h>`, `std::shared_ptr<SMS_STS>`, and `SMS_STS_*` register
constants.

### 4. Examples removed

Upstream ships example programs per protocol family (`examples/SMS_STS/`,
`examples/sandbox/`, etc.). This vendored copy only needs to build the
`SCServo` library for linking, so `examples/` was removed entirely. See the
upstream repo for example programs.

### 5. Repo-level tooling removed

These belong to the upstream repo's own CI/dev workflow, not to
`sts_hardware_interface` (which has its own equivalents at the package root):

- `.clang-format`, `.clang-tidy`, `.pre-commit-config.yaml`, `.gitignore`
- `.github/` (CI workflows)
- `cmake/` and the associated CMake package-export/install machinery in
  `CMakeLists.txt` (`SCServoConfig.cmake.in`, `install(EXPORT ...)`, etc.) —
  unused here since this is built via `add_subdirectory()`, never
  `find_package(SCServo)`
- `LICENSE` — see `sts_hardware_interface`'s own license instead
- `docs/` and `README.md` — see the upstream links at the top of this file

### 6. Build artifact cleanup

Upstream's git history had ~39 compiled example binaries and a stray
`.gitmodules` (referencing an unrelated, never-tracked `build/extern/nanobind`
Python-bindings experiment) accidentally committed. Neither was carried into
this vendored copy.

## What did *not* change

The actual servo protocol implementation — packet construction, checksums,
register maps, read/write/sync-write logic — is untouched. `SMS_STS.h`,
`SMS_STS.cpp`, `SCS.cpp`, and `SCSerial.cpp` are byte-for-byte identical to
upstream. `SCS.h` and `SCSerial.h` only have doc-comment edits (removing
references to the deleted `SCSCL`/`HLSCL` classes, see §2) — no functional
change. This is a structural trim (fewer servo families, no examples, no repo
tooling), not a functional fork.
