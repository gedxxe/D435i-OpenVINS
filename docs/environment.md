# Development environments observed through 2026-07-26

The repository was initially created and statically audited in a Windows 11
desktop session:

- OS: Microsoft Windows NT 10.0.26200.0
- PowerShell: 5.1.26100.8894
- Git: 2.54.0.windows.1
- GCC/G++: MSYS2 12.2.0
- Python: 3.13.12
- CMake, Ninja, pkg-config, OpenCV, Eigen, Ceres, and librealsense2: not
  available on PATH
- physical D435i: not accessible/verified in this session

Consequently this record is not the required Ubuntu 24.04 environment report.
The supported build creates the current report in `.deps/environment.txt`.

An operator then executed the scripts from the same tree on Ubuntu 24.04.4 LTS
x86-64 and supplied the complete console output. That run observed kernel
7.0.0-28-generic, GCC 13.3.0, CMake 3.28.3, OpenCV 4.6.0, Eigen 3.4.0, and
Python 3.12.3. The dependency installer completed successfully with all
requested packages already installed.

The first full build did not complete: Ceres 2.1.0 detected Ubuntu's CXSparse
4.3.1 but generation failed because `CXSparse::CXSparse` was unavailable.
The repository build now disables the unused SuiteSparse/CXSparse backends and
validates those cache values. A second Ubuntu run confirmed that Ceres
configured, generated, built, and installed successfully with both backends
disabled. That run then stopped before librealsense configuration because
Windows Git had converted dependency working-tree text to CRLF on the shared
NTFS volume. Git reported 388 OpenVINS files and 2,909 librealsense files even
though comparison ignoring only CR-at-EOL found no substantive changes.

A third Ubuntu run confirmed that the CRLF-aware integrity check accepts the
unchanged OpenVINS checkout. It built and installed local Ceres 2.1.0 and
librealsense 2.56.5, then configured OpenVINS without ROS against Ceres 2.1.0
from `.deps/install/ceres`. The build stopped before OpenVINS compilation
because the repository validator expected `Ceres_DIR:PATH` while this
command-line cache entry was correctly stored as `Ceres_DIR:UNINITIALIZED`.
Cache checks now compare exact values independently of CMake's type
serialization.

That run also showed that librealsense's defaults downloaded a recommended
firmware bundle, built unnecessary tools, and attempted a global `ldconfig`
during a repository-local install. The current build disables firmware import,
tools, examples, tests, update checks, graphical/Python extensions, and the
all-in-one static bundle. It supplies a private no-op `ldconfig` only for that
local install call. It neither invokes a firmware updater nor changes the
global loader cache.

The subsequent manual `ctest` invocation found no tests because the dependency
build had still not configured `build/linux-release`; it was not a test pass.
The supported command names that build directory and treats no tests as an
error.

Later operator output confirmed that hardware-enabled `ovrs_inspect` and
`ovrs_record` launch and can identify the connected physical D435i. An initial
connection negotiated USB 2.1 and exposed only reduced stereo combinations.
The first follow-up source fix added explicit inspector profile overrides,
kept automatic fallback disabled, and delayed recorder dataset initialization
until the requested camera profile had started.

A newer `rs-enumerate-devices` run reports USB 3.2 and advertises the configured
IR1/IR2 `848x480@30`, alternative `640x480@30`, 200/400 Hz gyro, and 63/250 Hz
accelerometer profiles. The attempted inspector still rejected `--width`,
proving that the Ubuntu executable predated the source fix. The recorder
accepted the override but stopped with `failed to set power state`.

The v0.3.1 source now uses one librealsense context for device selection and
pipeline construction, and uses the SDK device-change callback instead of
calling `query_devices()` repeatedly during an active stream. A fresh Windows
portable audit build under `build/audit-v0.3.1-runtime-fix-20260725` compiled
all six dependency-disabled targets with `-Werror`; core tests passed 15/15
and synthetic replay passed 1/1. Integration-enabled syntax checks for the
RealSense source, inspector, recorder, and live entry point passed against the
pinned v2.56.5 headers. These checks do not establish successful Ubuntu camera
sampling.

The next Ubuntu preflight completed with zero errors and one warning: system
librealsense 2.58.3 is present, but the project correctly resolves its
repository-local 2.56.5. The physical D435i then completed a 60.55-second
848x480 Y8 recording. At that point the retained dataset contained 1,796
stereo pairs and 11,980 synchronized IMU samples, reported Global Time for
every stream and zero drops, declared `complete: true`, and had no
`INCOMPLETE` marker.

Calibration export then exposed an inspector-only bug: capability enumeration
requested intrinsics from a Y16 1280x800 profile that does not provide them.
The current fix queries intrinsics only for Y8 calibration profiles, tolerates
unavailable non-target intrinsics, and validates both selected profiles before
export. Live/replay now also reject camera-calibration resolution mismatches.
Hardware preflight now requires the project inspector to complete a one-second
sample instead of accepting device enumeration alone. These latest fixes are
pending an Ubuntu rebuild and physical inspector rerun.

The first retry still executed the old enumeration logic even though
`--version` reported v0.3.1. Binary inspection confirmed that the Ubuntu
executable did not contain the new intrinsics-handling strings. On the shared
NTFS checkout, generated objects had timestamps later than the edited source,
so a semantic-version check alone was insufficient. v0.3.2 fingerprints the
content of all project C++/header/CMake inputs, embeds that identity in every
executable, and makes both build and preflight reject a mismatch.

The rebuilt physical inspector subsequently listed Y16 profiles without
aborting, selected both 848x480 Y8 cameras, sampled for 10.4 seconds, and
reported zero malformed frames, rejected timestamps, and callback errors.
All streams remained in Global Time and the serial-specific
`BOOTSTRAP_UNVERIFIED` factory export was written. The current shared
`datasets/walk_loop_001` now contains a separate complete 10.43-second
recording with 297 stereo pairs and 1,982 synchronized IMU samples. The audit
does not know when the earlier ignored 60-second directory was removed.

Repository-local checks completed on this host:

- the v0.3.0 re-audit created fresh manual GCC outputs under
  `build/audit-v0.3.0-20260725` without deleting prior builds;
- the Ubuntu-fix follow-up created a second fresh portable build under
  `build/audit-ubuntu-fix-20260725`; all six targets compiled with `-Werror`,
  both test suites passed, and all four help/version pairs passed;
- the v0.3.1 runtime-fix audit created a new build directory, compiled all six
  dependency-disabled targets with `-Werror`, passed 15/15 core tests and 1/1
  mock replay test, and reported v0.3.1 from every CLI;
- the inspector/calibration follow-up compiled all six targets under
  `build/audit-intrinsics-fix-20260725`, repeated both test suites, tested
  calibration resolution validation, and passed hardware inspector syntax
  against the pinned librealsense headers;
- the v0.3.2 identity follow-up compiled all six dependency-disabled targets
  under `build/audit-v0.3.2-fingerprint-20260725` with `-Werror`, passed
  15/15 core tests and 1/1 mock replay test, and verified every CLI's
  version/fingerprint output. The shared shell identity helper accepted the
  correct fingerprint and rejected a deliberately wrong one;
- project-owned core, both tests, and all four dependency-disabled CLI targets
  compiled as C++17 with `-Wall -Wextra -Wpedantic -Werror`;
- `ovrs_core_tests`: 15/15 passed;
- `ovrs_mock_replay_test`: 1/1 passed;
- the earlier v0.3.0 and v0.3.1 CLI `--help`/`--version` checks returned 0
  and reported their respective semantic versions; v0.3.2 validation is
  documented separately because it also checks source content identity;
- all four CLIs rejected unknown options with code 2, and missing option values
  were also rejected;
- RealSense source, inspector, recorder, live entry point, and replay entry
  point passed integration-enabled syntax checks using the pinned v2.56.5
  headers; OpenCV-only syntax used a local audit stub, not an OpenCV runtime;
- all Bash scripts passed `bash -n`;
- `plot_trajectory.py` passed Python 3.13.12 byte-compilation, summary execution,
  and PNG generation (the available global Matplotlib was 3.10.8, not the
  pinned optional venv);
- all three upstream OpenVINS Python/launch files passed Python 3.13 syntax;
  their ROS imports were not available and are outside the standalone build;
- `CMakePresets.json` parsed successfully and all project-local include/CMake
  source references existed;
- exact OpenVINS and ignored local librealsense checkout revisions matched the
  central pins; OpenVINS had no tracked modifications;
- the read-only preflight failed closed on this non-Linux host, as designed.

A physical stereo/IMU recording and a serial-specific 848x480 factory export
are now verified from the operator's Ubuntu run. The export remains
`BOOTSTRAP_UNVERIFIED`; acknowledged local-bundle creation, registered CTest
output after the latest edits, replay, live OpenVINS initialization, and
sanitizer execution remain unverified. Repository checks share one implementation that
ignores only executable-mode metadata and CR immediately before LF. Binary
changes, other whitespace changes, and substantive text changes still fail
closed. No physical D435i was accessible from the Windows audit process; the
hardware evidence came from the operator's Ubuntu run and the resulting shared
dataset.

## Current v0.3.3 Windows re-audit

The source was audited again on 2026-07-26 after the operator/run-integrity
changes. The latest new directory,
`build/audit-v0.3.3-readme-20260726`, was used;
existing builds were not deleted.

- GCC 12.2 compiled all six dependency-disabled project targets as C++17 with
  `-Wall -Wextra -Wpedantic -Werror`.
- Core tests passed 16/16 and synthetic replay passed 1/1.
- All four CLIs passed `--help` and reported v0.3.3 with source fingerprint
  `a641b8e40ddd7f865b7beb818ad633e3b450629cb804c27ef6c3d0e0aada3fff`.
- Replay rejected `--serial` with exit 2 and explained that identity comes
  from dataset metadata.
- The current RealSense source and dependency-enabled inspect, record, live,
  and replay entry points passed strict syntax checks against the pinned
  librealsense 2.56.5 headers. The OpenCV portion used an ignored audit stub,
  so this is not an OpenCV compile or link result.
- All 9 repository shell scripts passed `bash -n`. All 16 README Bash blocks
  passed syntax parsing; the manual-test document is now evidence-only and
  contains no competing command procedure.
- In the earlier v0.3.x tree, CPython 3.13.12 parsed all four then-present
  repository/submodule Python files. The
  optional plotter completed summary and PNG smoke tests using the available
  global Matplotlib 3.10.8; this did not install or modify a venv.
- The factory validator accepted the supplied 848x480 export for the
  operator's D435i serial (redacted from tracked documentation) and rejected
  both an incorrect expected serial and a 640x480 export paired with the
  848x480 stream file.
- The preflight parser passed its help and option-coupling checks and reached
  its expected non-Linux failure for a syntactically valid camera request.

CMake, Ninja, Docker, OpenCV, Eigen, Ceres, and librealsense are still not on
the current Windows PATH. The WSL launcher is present but unavailable in this
session. Therefore v0.3.3 CMake configure/link, registered CTest, sanitizers,
Ubuntu execution, physical camera sampling, reviewed-bundle replay, and live
OpenVINS initialization remain pending. Historical physical evidence above
belongs to the earlier v0.3.2-capable Ubuntu binary and must not be treated as
a v0.3.3 hardware pass.

## v0.4.0 estimator-evidence audit

The physical 2026-07-25 recording and replay/live outputs were inspected after
the operator confirmed the test motion was a slow rotation and return near the
start. Replay displacement 660.730498 m and live displacement 33.184267 m are
invalid results, not successful VIO validation.

On the first operator-confirmed stationary three-second interval, the recorded
mean acceleration norm is 9.415342258 m/s² versus configured gravity
9.806650000 m/s². One orientation cannot determine a full accelerometer
intrinsic matrix, bias random walk, or camera-IMU temporal calibration, so no
numeric calibration was fabricated.

The Windows host validated v0.4.0 portable code with GCC 12.2 strict warnings:
17/17 core cases and 1/1 synthetic replay passed. CPython 3.13.12
byte-compiled both project scripts, the stationary analyzer reproduced the
gravity mismatch, the known invalid replay failed an operator-supplied
displacement bound, and PNG generation was visually inspected. The current
RealSense source passed syntax checking against pinned 2.56.5 headers.
All 10 current repository shell scripts passed `bash -n`. The serial-specific
ignored bundle was migrated without numeric changes so its IMU YAML now
carries the same `BOOTSTRAP_UNVERIFIED` state, serial, and motion-correction
policy as the main configuration; the pre-migration IMU file was retained.

OpenCV HighGUI, the dependency-enabled OpenVINS applications, Ubuntu
`python3-tk`, and physical D435i operation of the current source were not
available on Windows. They require a clean v0.4.0 Ubuntu rebuild and must not
be claimed from these portable checks.

## v0.5.0 calibration-contract audit

The current Windows host confirmed that pinned OpenVINS v2.7 requests
`T_imu_cam` and has an explicit fallback that reads and inverts Kalibr's
`T_cam_imu`. The earlier key was therefore compatible, not ignored. The
project-owned contract, exporter, validators, migration, promotion, tests, and
documentation were canonicalized so runtime files use one direction.

MSYS2 GCC 12.2 compiled the v0.5.0 portable core and all four
dependency-disabled CLI stubs with `-Wall -Wextra -Wpedantic`. Core tests
passed 18/18 and synthetic replay passed 1/1. Python 3.13.12 passed nine
calibration workflow tests, including provenance-tamper, Allan-rate, and
stationary-gravity-bound rejection, byte-compilation, and all calibration CLI
help checks. Export v2
copies and hashes source metadata. The RealSense source now rejects
non-D435i models; it and the hardware-enabled inspector passed syntax
checking against the pinned 2.56.5 headers; the recorder passed with those
headers plus a local OpenCV API stub. These were not link/runtime tests. Git
Bash parsed all 10 repository shell scripts and 33 README Bash blocks with
`bash -n`.

A final fresh rebuild under
`build/audit-v0.5.0-final2-20260726` repeated the 18/18 core result, 1/1
synthetic replay result, and four dependency-disabled CLI help/version checks.
The plotter passed finite bounds, generated a visually inspected PNG, and
returned its expected failure code for a deliberately too-small displacement
bound. All 10 tracked configuration YAML files parsed. The live
application, including its integrity-counter gate, passed syntax checking with
both feature macros enabled. The viewer/estimator implementation build and
final link still require Ubuntu.

The dependency-enabled recorder/RealSense changes, full CMake graph, OpenCV,
OpenVINS link, Ubuntu execution, and physical D435i capture remain unverified
for v0.5.0 because CMake and hardware dependencies are unavailable on the
current Windows host. Docker is not installed or assumed. WSL distro
enumeration was denied.
