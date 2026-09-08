# AMCEO Stage 2A Build Baseline

> Audit date: 2026-09-07 (Asia/Shanghai)  
> Repository: `/home/linyi666/PX4-Autopilot-water-air`  
> Revision: `6abec104d4680a2c2c36499d665aec7ab18a3063` (`AMCEO`, `v1.15.0-beta1-345-g6abec104d4`)  
> Target: `matek_h743_dummy`  
> Scope: build-environment recovery and baseline compilation only

## 1. Repository Build Requirements

### Framework, target, and command

This is a customized PX4 Autopilot tree using CMake/Ninja and NuttX for the
Matek H743 Cortex-M7 target. The target configuration is
`boards/matek/h743/dummy.px4board`; the correct command is:

```sh
make matek_h743_dummy
```

The target name is not `matek_h743_default`. The final CMake cache records
`PX4_CONFIG=matek_h743_dummy` and the dummy board file.

### Repository-declared compiler

`Tools/setup/ubuntu.sh` is the strongest in-repository version declaration:

- it states support for Ubuntu LTS 18.04, 20.04, and 22.04;
- it sets `NUTTX_GCC_VERSION="9-2020-q2-update"`;
- it sets `NUTTX_GCC_VERSION_SHORT="9-2020q2"`;
- it downloads the matching GNU Arm Embedded archive and prepends its `bin`
  directory to `PATH`.

The actual compiler in that archive identifies itself as:

```text
arm-none-eabi-gcc (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1 20200408 (release)
```

`platforms/nuttx/cmake/Toolchain-arm-none-eabi.cmake` selects the unversioned
`arm-none-eabi-gcc`, `arm-none-eabi-g++`, and related tools from `PATH`; it does
not enforce or validate a version. Correct PATH ordering is therefore part of
the build environment.

### Repository-declared Python environment

There is no single pinned Linux Python minor version in this revision:

- `Tools/setup/ubuntu.sh` installs `python3`, `python3-dev`, `python3-pip`,
  `python3-setuptools`, and `python3-wheel` on supported Ubuntu releases;
- the supported Ubuntu releases imply multiple distribution Python versions,
  rather than one universal pin;
- `.github/workflows/compile_macos.yml` explicitly selects Python 3.10;
- CMake requires Python 3 and selected `/usr/bin/python3` in the verified
  build.

Accordingly, Python 3.10 has the clearest explicit workflow pin, while the
Linux setup contract is the distribution `python3` from Ubuntu 18.04/20.04/
22.04. The audited Python 3.12 host is outside that stated Ubuntu matrix even
though this target successfully compiled with it.

Python dependencies are split as follows:

- `Tools/setup/requirements.txt`: the normal setup set, including
  `empy==3.3.4`, `sympy>=1.10.1`, NumPy, Jinja2, kconfiglib, pymavlink,
  pyserial, pyulog, and other build utilities;
- `Tools/setup/optional-requirements.txt`: only `symforce>=0.9.0`.

This distinction matters: `ubuntu.sh` installs `requirements.txt`, not the
optional file.

### Container, devcontainer, CI, and documentation audit

The repository's NuttX CI baseline is consistently pinned to:

```text
px4io/px4-dev-nuttx-focal:2022-08-12
```

It is used by `.github/workflows/compile_nuttx.yml` and
`.github/workflows/nuttx_env_config.yml`, and also appears in `checks.yml`,
`failsafe_sim.yml`, and `metadata.yml`. `Tools/docker_run.sh` defaults to the
same image for NuttX targets.

The local `.devcontainer/Dockerfile` is a project customization based on
`px4io/px4-dev-ros-noetic:2023-06-26`. It runs `Tools/setup/ubuntu.sh` and later
installs extra packages including SymForce. `.devcontainer/devcontainer.json`
builds that Dockerfile and requests privileged/device access. It is not as
tightly scoped to headless NuttX CI as the pinned `px4-dev-nuttx-focal` image.

The root README identifies PX4 and links to the external Developer Guide but
does not add a local compiler/Python pin. No checked-in local docs override the
versions in `Tools/setup/ubuntu.sh` or the CI container declaration.

## 2. Current Environment

### Host

- OS: Ubuntu 24.04.4 LTS (`noble`) under WSL2
- Kernel: `6.6.87.2-microsoft-standard-WSL2`, x86_64
- Host release status relative to repository: newer than the setup script's
  stated Ubuntu 18.04/20.04/22.04 support set

### ARM toolchains before recovery

`type -a` found only:

```text
/usr/bin/arm-none-eabi-gcc
/bin/arm-none-eabi-gcc
/usr/bin/arm-none-eabi-g++
/bin/arm-none-eabi-g++
```

`/bin` and `/usr/bin` resolve to the same installed toolchain. Its version was:

```text
arm-none-eabi-gcc (15:13.2.rel1-2) 13.2.1 20231009
arm-none-eabi-g++ (15:13.2.rel1-2) 13.2.1 20231009
```

No alternate `arm-none-eabi-gcc` was found under `/opt`, `/usr/local`,
`/home/linyi666/.local`, or the existing virtual-environment directory before
the recovery.

### Python and environment managers

- Available Linux Python executables: Python 3.12.3 via `/usr/bin/python3` and
  `/usr/bin/python3.12`; no Linux Python 3.8, 3.9, 3.10, or 3.11 was found.
- Existing venv: `/home/linyi666/.venvs/px4-waterair`, also Python 3.12.3.
- `virtualenv` is installed at `/home/linyi666/.local/bin/virtualenv`.
- No active venv or Conda environment was present (`VIRTUAL_ENV` and
  `CONDA_PREFIX` unset).
- No Linux `pyenv`, Conda, or micromamba command was found. A Windows Anaconda
  tree is present through `/mnt/d`, but its Windows executables are not a Linux
  PX4 build environment and were not used.

System/user-site packages relevant to the observation were:

- `symforce==0.11.0` from
  `/home/linyi666/.local/lib/python3.12/site-packages`;
- `sympy==1.14.0`;
- `empy==3.3.4`;
- `nunavut==2.3.1`.

The existing `px4-waterair` venv independently contains the same SymForce and
SymPy versions and exhibits the same module-execution traceback.

### Container availability

`docker` was not installed (`docker: command not found`), and no Podman command
was available. Therefore neither the pinned CI image nor the checked-in
devcontainer could be used on this machine without a larger host-level Docker
installation. Stage 2A did not make that invasive change.

### Cache environment

PX4 detected `/usr/bin/ccache`. Its default cache directory was
`/home/linyi666/.cache/ccache`, which was read-only in the current managed
execution environment. A build using that default stopped immediately when
the first compiler jobs attempted cache writes. The verified build set only
`CCACHE_DIR=/tmp/amceo-stage2a-ccache`.

## 3. Root Cause of Previous Failure

### Previous Stage 1 fatal error

The previous command was `make matek_h743_dummy -j2` with GCC 13.2.1. The first
true fatal source diagnostic was:

```text
platforms/common/spi.cpp:90:59: error: the address of 'px4_spi_buses'
will never be NULL [-Werror=address]
cc1plus: all warnings being treated as errors
```

Classification: **environment/toolchain compatibility failure expressed as an
unrelated platform-source compile failure**. The newer compiler emits an
`-Waddress` diagnostic for source that the repository-declared GCC 9 accepts;
the existing `-Werror` policy correctly promotes it to fatal. The successful
GCC 9 build proves this was not a CADRC or custom hydro module failure.

No source line, warning flag, CMake file, or board configuration was changed to
resolve it.

### SymForce diagnostic

The traceback can be reproduced exactly with:

```sh
python3 -m symforce.symbolic
```

With Python 3.12.3 and user-site `symforce==0.11.0`, module execution imports
`symforce.util` and `symforce.codegen` back into a partially initialized
`symforce.symbolic`, ending at:

```text
ImportError: cannot import name 'lambdify' from partially initialized module 'symforce.util'
```

This is not the fatal baseline-build root cause. In
`src/modules/ekf2/CMakeLists.txt`, `execute_process()` stores the probe return
code in `PYTHON_SYMFORCE_EXIT_CODE`; subsequent optional SymForce generation is
enabled only when that code equals zero. The failed probe emits stderr but
configuration continues using checked-in generated EKF sources. Both verified
GCC 9 builds completed with the traceback present.

### Stage 2A transient launcher/cache failures

For completeness, two environment-only failures occurred while establishing
the repeatable command:

1. `env PATH=...:$PATH` was passed through `script -c`; spaces in the WSL-imported
   Windows PATH were split by the second shell, so `env` exited 127 before
   `make` ran. Quoting PATH in an `export` removed this launcher error.
2. The first compiler attempt with GCC 9 stopped at 22/1058 with
   `ccache: error: Read-only file system`. The first failed object was
   `src/lib/crc/crc.c.obj`, but no compiler diagnostic was produced. Setting a
   writable cache directory removed the environment failure.

Neither event is a generated-file, source, or hydro-module failure.

## 4. Compatible Environment Selected

The preferred pinned CI container (priority A) could not run because Docker is
absent. The selected priority-B environment is therefore:

- the exact standalone GNU Arm Embedded toolchain declared by
  `Tools/setup/ubuntu.sh`;
- installed without sudo at
  `/home/linyi666/.local/opt/gcc-arm-none-eabi-9-2020-q2-update`;
- Python 3.12.3 from `/usr/bin/python3`, because it is the only available Linux
  Python and was sufficient for the complete target build;
- ccache redirected to the writable `/tmp/amceo-stage2a-ccache`.

The downloaded archive SHA-256 is:

```text
5adc2ee03904571c2de79d5cfc0f7fe2a5c5f54f44da5b645c17ee57b217f11f
```

An attempted installation to the repository script's `/opt` location was not
performed because sudo required an unavailable interactive password. The
user-level installation contains the identical verified archive and avoids
changing global compiler selection.

The successful ELF confirms the compiler independently of PATH/CMake logs:

```text
GCC: (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1 20200408 (release)
```

## 5. Exact Reproduction Steps

The persistent environment entry is documented separately in
`AMCEO_BUILD_ENVIRONMENT.md`. For this machine:

```sh
cd /home/linyi666/PX4-Autopilot-water-air

export PATH="/home/linyi666/.local/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"
export CCACHE_DIR=/tmp/amceo-stage2a-ccache
mkdir -p "$CCACHE_DIR"

arm-none-eabi-gcc --version
python3 --version
make matek_h743_dummy
```

For the final verified clean configuration, the previous generated target
directory was moved out of the repository before the command:

```sh
mv build/matek_h743_dummy /tmp/amceo-stage2a-success-temp-path-matek_h743_dummy
make matek_h743_dummy
```

The exact compiler selected by generated CMake metadata is
`/home/linyi666/.local/opt/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-gcc`;
the C++ compiler is the adjacent `arm-none-eabi-g++`. The exact interpreter in
`CMakeCache.txt` is `/usr/bin/python3`.

## 6. Baseline Build Result

Result: **SUCCESS (exit code 0), 1058/1058**.

The first clean GCC 9 build with an empty writable ccache completed in:

```text
real 51.81 s
user 304.40 s
sys  87.10 s
```

After installing the same toolchain at its stable user-level path, a second
clean target configuration/build completed successfully with a warm cache:

```text
real 26.61 s
user 40.30 s
sys  12.55 s
```

The latter is the build that produced the final artifacts below. Its cache had
1,789 direct hits; use 51.81 seconds as the closer cold-cache baseline.

Key completion evidence:

- `hydro_att_control` linked;
- `hydro_rate_control` linked;
- `hydro_allocator` linked;
- final `matek_h743_dummy.elf` linked;
- final `.bin` generated;
- final `.px4` package created.

Linker memory report:

- FLASH: 1,582,200 bytes / 1,920 KiB, 80.47%;
- AXI SRAM: 44,652 bytes / 512 KiB, 8.52%.

The SymForce traceback described above remained visible but was non-fatal. No
warning was disabled and `-Werror` remained in compiler commands.

## 7. Firmware Artifact Information

All timestamps are Asia/Shanghai (`+0800`).

| Artifact | Size | Timestamp | SHA-256 |
|---|---:|---|---|
| `build/matek_h743_dummy/matek_h743_dummy.px4` | 1,523,513 B | 2026-09-07 17:02:10.616616424 +0800 | `a952749c6f24a61fc500389422ae732ff90815b63aa57f1d5c7d0c0f1fe351ae` |
| `build/matek_h743_dummy/matek_h743_dummy.elf` | 36,611,064 B | 2026-09-07 17:02:10.469283125 +0800 | `74ed20fcbeb0ec480da9d8edfeab22a1e38ad0d5e846f0473a59c92a6da1822f` |
| `build/matek_h743_dummy/matek_h743_dummy.bin` | 1,582,200 B | 2026-09-07 17:02:10.508283116 +0800 | `70fce014f5b610e3432fe57d040217660a12ceacd2cc1f416cfc4664a5f9982d` |

The requested absolute artifact paths are:

```text
/home/linyi666/PX4-Autopilot-water-air/build/matek_h743_dummy/matek_h743_dummy.px4
/home/linyi666/PX4-Autopilot-water-air/build/matek_h743_dummy/matek_h743_dummy.elf
```

The `.px4` is a JSON-format PX4 package. The `.elf` is a 32-bit little-endian
ARM EABI5 executable, statically linked, with debug information and not
stripped.

## 8. Git Working Tree Verification

Before environment work and before any target-directory movement:

```text
?? AMCEO_INTEGRATION_BASELINE_AUDIT.md
```

`git ls-files -m` was empty. Thus no tracked source/configuration file was
modified at the start.

After the successful build and before writing the two Stage 2A documents:

```text
?? AMCEO_INTEGRATION_BASELINE_AUDIT.md
```

Again, `git ls-files -m`, `git diff --stat`, and
`git diff --submodule=short --exit-code` showed no tracked changes. The build
directory is generated/ignored. The final status adds only these two requested
untracked documentation files:

```text
?? AMCEO_BUILD_ENVIRONMENT.md
?? AMCEO_INTEGRATION_BASELINE_AUDIT.md
?? AMCEO_STAGE2A_BUILD_BASELINE.md
```

No existing tracked file was edited, moved, deleted, or renamed. Specifically,
`platforms/common/spi.cpp`, warning flags, CMakeLists, board configuration,
CADRC, HydroRateControl, HydroAttControl, HydroAllocator, parameters, and all
control logic remain unchanged.

Generated build directories moved to `/tmp` for preservation were:

- the pre-Stage-2A target directory;
- the partial build stopped by read-only ccache;
- the first successful GCC 9 target directory before the stable-path rebuild.

The final valid generated target directory remains at
`build/matek_h743_dummy`.

## 9. Remaining Environment Risks

1. **Python 3.12 is outside the repository's stated Ubuntu matrix.** It built
   this target successfully, but future generators or tooling may expose other
   compatibility issues.
2. **SymForce module execution is noisy/broken on this host.** The optional
   probe fails with `symforce==0.11.0`; this baseline safely falls back to
   checked-in generated EKF sources, but any target requiring regeneration must
   not assume the probe is usable.
3. **Python dependencies are lower-bound-heavy.** Except for Empy, most
   requirements are not fully pinned, so recreating with current packages at a
   later date may not be byte-for-byte equivalent.
4. **The CI container is unavailable locally.** Installing Docker later would
   enable the strongest repository-pinned environment, but also introduces a
   separate host administration task.
5. **ccache's default directory is read-only in the managed session.** Omitting
   `CCACHE_DIR` reproduces an environment failure before meaningful compiler
   diagnostics.
6. **PATH can silently select GCC 13.** Every clean configuration must verify
   `arm-none-eabi-gcc --version`; an already configured build also retains the
   absolute compiler path in CMake metadata.
7. **WSL-imported PATH contains spaces.** Passing an unquoted expanded PATH
   through nested shells can fail before `make`; use the documented quoted
   `export`.
8. **Firmware hashes may not be intrinsically reproducible across build times
   or paths.** This stage records exact artifacts but did not claim a
   reproducible-build/bit-identical property.
9. **Toolchain is user-local rather than `/opt`.** This is deliberate because
   sudo was unavailable. Other users/agents must use the exact documented path
   or independently install the verified archive.

## 10. Recommendation for Stage 2B

Use this successful GCC 9 build and the recorded artifacts as the immutable
pre-change baseline for Stage 2B. At the start of every Stage 2B shell:

1. apply the exports in `AMCEO_BUILD_ENVIRONMENT.md`;
2. verify GCC reports `9-2020-q2-update` and CMake does not retain a GCC 13
   path;
3. record `git status --short` and `git ls-files -m`;
4. preserve the Stage 2A `.px4`/`.elf` hashes and build log metadata before any
   authorized source work;
5. keep the common limiter/allocation path and existing CADRC branch unchanged
   unless a later stage explicitly authorizes a narrowly scoped change;
6. rebuild `matek_h743_dummy` after each bounded change and distinguish
   environment, generated-file, generic source, and custom hydro failures by
   the first fatal diagnostic.

Do not treat the non-fatal SymForce traceback as a CADRC failure. If Stage 2B
requires EKF symbolic regeneration, stop and establish a separately validated
Python environment first; normal controller integration should not expand into
that unrelated task.
