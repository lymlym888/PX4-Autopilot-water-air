# AMCEO Baseline Build Environment

This file is a non-runtime build note for revision
`6abec104d4680a2c2c36499d665aec7ab18a3063`. It does not change PX4, board,
parameter, or controller behavior.

## Selected environment

- Target: `matek_h743_dummy`
- Repository-declared Arm toolchain: GNU Arm Embedded
  `9-2020-q2-update` (GCC 9.3.1, 20200408 release)
- Installed user-level toolchain:
  `/home/linyi666/.local/opt/gcc-arm-none-eabi-9-2020-q2-update`
- Python used by the verified build: `/usr/bin/python3`, Python 3.12.3
- Writable ccache location: `/tmp/amceo-stage2a-ccache`

The GCC version is taken directly from `Tools/setup/ubuntu.sh`. The repository
does not pin one Linux Python minor version: that script supports Ubuntu 18.04,
20.04, and 22.04 and requests `python3`; the macOS CI workflow explicitly uses
Python 3.10. Python 3.12.3 succeeded for this target, but remains newer than the
repository's declared Ubuntu matrix.

## Shell activation

Run these commands in every new shell before configuring or building this
target:

```sh
export PATH="/home/linyi666/.local/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"
export CCACHE_DIR=/tmp/amceo-stage2a-ccache
mkdir -p "$CCACHE_DIR"

arm-none-eabi-gcc --version
python3 --version
```

The compiler version must begin with:

```text
arm-none-eabi-gcc (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1 20200408 (release)
```

The separate `CCACHE_DIR` is required on the audited host because the default
`/home/linyi666/.cache/ccache` is read-only in the current execution
environment. This changes only cache storage, not compiler flags or output
semantics.

## Build

Normal incremental build:

```sh
cd /home/linyi666/PX4-Autopilot-water-air
make matek_h743_dummy
```

For a clean target reconfiguration without deleting tracked files, first move
the generated target directory to a uniquely named location outside the
repository, then build:

```sh
cd /home/linyi666/PX4-Autopilot-water-air
mv build/matek_h743_dummy /tmp/matek_h743_dummy.previous
make matek_h743_dummy
```

Only run the `mv` command when `build/matek_h743_dummy` exists and when the
destination does not already exist. Moving the directory is preferred here to
preserve the old generated artifacts for inspection.

## Toolchain bootstrap on another matching Linux host

The URL below is copied from the version and URL pattern in
`Tools/setup/ubuntu.sh` for x86_64:

```sh
wget -O /tmp/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 \
  https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2
sha256sum /tmp/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2
mkdir -p /home/linyi666/.local/opt
tar -xjf /tmp/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 \
  -C /home/linyi666/.local/opt
```

Expected archive SHA-256:

```text
5adc2ee03904571c2de79d5cfc0f7fe2a5c5f54f44da5b645c17ee57b217f11f
```

The repository-preferred alternative is its pinned NuttX CI image,
`px4io/px4-dev-nuttx-focal:2022-08-12`, via `Tools/docker_run.sh`. Docker was
not installed on the audited host, so the verified baseline uses the standalone
repository-declared toolchain instead.

## Known non-fatal Python diagnostic

This host has user-site `symforce==0.11.0`. During configure,
`src/modules/ekf2/CMakeLists.txt` probes it with:

```sh
python3 -m symforce.symbolic
```

That invocation emits a circular-import traceback. In the current source, the
return code gates optional SymForce regeneration; it does not stop the normal
target build. The baseline completed despite this diagnostic. Do not modify
EKF2 CMake or generated estimator sources to hide it. A future environment
hardening task may use an isolated, repository-supported Python environment,
but must revalidate generated-source behavior before adoption.

