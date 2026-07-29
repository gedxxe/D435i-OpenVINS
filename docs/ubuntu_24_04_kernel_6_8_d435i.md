# Ubuntu 24.04 kernel 6.8 recovery runbook for D435i IMU

This runbook is for the specific failure pattern where a D435i is enumerated,
stereo IR capture works, but gyroscope and accelerometer sampling both remain
at zero. It explains how to test the Ubuntu 24.04 GA kernel 6.8 without
removing the current HWE kernel.

Do not use this procedure for an ordinary calibration-quality problem. Kernel
selection cannot correct IMU noise, bad camera-IMU extrinsics, time offset, or
an inaccurate Allan fit.

## Why this test exists

The affected host was observed with:

- Ubuntu 24.04.4 LTS;
- kernel `7.0.0-28-generic`;
- Secure Boot enabled;
- NVIDIA open driver branch 595;
- D435i firmware `5.17.3.10`;
- D435i visible over USB 3.2;
- stereo-only capture working;
- IMU-only capture producing no frames;
- the official low-level `rs-data-collect` tool also producing no IMU frames;
- Linux IIO accel/gyro raw reads timing out.

That evidence places the failure below the OpenVINS and project callback
layers. Librealsense v2.56.5 lists Ubuntu kernels through 6.8 among its
supported platforms; kernel 7.0 is not in that release's supported-kernel
list. Testing the stock Ubuntu 24.04 GA kernel is therefore a controlled host
compatibility experiment, not proof that kernel 7.0 is the cause.

Primary references:

- [librealsense v2.56.5 release and supported platforms][librealsense-release]
- [official librealsense Ubuntu installation guidance][librealsense-install]
- [Canonical guidance for installing a generic kernel][ubuntu-generic]
- [Ubuntu GRUB kernel submenu guidance][ubuntu-grub-submenu]

[librealsense-release]: https://github.com/realsenseai/librealsense/releases/tag/v2.56.5
[librealsense-install]: https://github.com/realsenseai/librealsense/blob/master/doc/installation.md
[ubuntu-generic]: https://documentation.ubuntu.com/real-time/latest/how-to/switch-from-realtime-to-generic-kernel/
[ubuntu-grub-submenu]: https://help.ubuntu.com/community/Grub2/Submenus

## Safety contract

This procedure:

- installs Ubuntu-signed GA kernel packages alongside the current kernel;
- keeps kernel 7.0 available as the rollback boot;
- keeps Secure Boot enabled;
- does not patch or compile a kernel;
- does not change D435i firmware;
- does not write calibration to EEPROM;
- does not purge any kernel;
- does not run `apt autoremove`;
- does not make kernel 6.8 the permanent GRUB default before it is tested.

Installing a kernel and NVIDIA modules is still a material system change. Save
active work, close GPU jobs, and allow downtime for at least two reboots.

Stop and obtain a current package review if any command resolves a different
NVIDIA branch, removes the active kernel, removes GRUB or shim, or proposes a
driver transition from open to proprietary modules.

## Plan overview

| Gate | Required result | If it fails |
|---|---|---|
| A. Baseline | Ubuntu 24.04, current kernel and package state recorded | Stop; do not install |
| B. APT simulation | Kernel 6.8 is added and kernel 7.0 remains installed | Use Plan A |
| C. Installation | Kernel, initramfs, GRUB entry, and NVIDIA module exist | Use Plan B |
| D. One-time boot | `uname -r` begins with `6.8.` | Use Plan C |
| E. Host graphics | Desktop and `nvidia-smi` work | Use Plan D |
| F. D435i sampling | Stereo, gyro, and accel all produce data | Continue project README |
| G. IMU still zero | Low-level test confirms or rejects the failure | Use Plan E |

## Phase 0: prepare without changing packages

Open a terminal in the repository:

```bash
REPO_DIR="$(git rev-parse --show-toplevel)"
cd "${REPO_DIR}"
set -o pipefail
mkdir -p output/kernel-6.8-migration
```

The `output/` directory is ignored by Git. The following report is local host
evidence and must not be committed:

```bash
{
  date --iso-8601=seconds
  lsb_release -ds
  uname -a
  mokutil --sb-state
  df -h / /boot /boot/efi
  dpkg-query -W -f='${Package} ${Version}\n' \
    linux-generic-hwe-24.04 \
    nvidia-driver-595-open \
    linux-modules-nvidia-595-open-generic-hwe-24.04
  ls -1 /boot/vmlinuz-* /boot/initrd.img-*
} | tee output/kernel-6.8-migration/before.txt
```

Expected facts on the diagnosed host:

- the active kernel is `7.0.0-28-generic`;
- Secure Boot is enabled;
- `/boot/efi` and `/` have free space;
- kernel 7.0 and its NVIDIA module package are installed.

Do not proceed if `/` or `/boot` is nearly full. Free space using normal
user-data management first; do not solve a space problem by blindly deleting
kernel files from `/boot`.

## Phase 1: refresh metadata and simulate the exact transaction

Refreshing APT metadata does not install packages:

```bash
sudo apt-get update
```

Confirm that Ubuntu 24.04 currently resolves the GA kernel to 6.8:

```bash
apt-cache policy linux-generic linux-image-generic
```

The candidate version must begin with `6.8.`. Stop if it does not.

The observed machine uses NVIDIA open-driver branch 595. Confirm that this is
still true:

```bash
dpkg-query -W -f='${Package} ${Version}\n' nvidia-driver-595-open
apt-cache policy linux-modules-nvidia-595-open-generic
```

If `nvidia-driver-595-open` is not installed, do not substitute another package
name by guessing. Review the active driver first:

```bash
nvidia-smi
dpkg-query -W -f='${Package} ${Version}\n' 'nvidia-driver-*' 2>/dev/null
```

Run the mandatory dry simulation:

```bash
apt-get -s install \
  linux-generic \
  linux-modules-nvidia-595-open-generic \
  | tee output/kernel-6.8-migration/apt-simulation.txt
```

Read the `NEW`, `upgraded`, and `REMOVED` sections. On the diagnosed host the
simulation was expected to:

- add the Ubuntu 6.8 GA image, modules, extra modules, headers, and tools;
- add the matching NVIDIA 595 open module for kernel 6.8;
- update NVIDIA 595 packages from `595.71.05` to the repository candidate;
- keep kernel `7.0.0-28-generic`;
- keep the NVIDIA module for kernel 7.0;
- possibly remove the obsolete NVIDIA module for kernel 6.17.

The package operation is **not approved** if the simulation proposes removing
any of:

```text
linux-image-7.0.0-28-generic
linux-modules-7.0.0-28-generic
linux-modules-extra-7.0.0-28-generic
linux-modules-nvidia-595-open-7.0.0-28-generic
grub-efi-amd64-signed
shim-signed
```

Also stop if the simulation proposes removing the desktop environment,
switching away from NVIDIA branch 595 open, or removing a large unrelated
package set. Do not add `--allow-downgrades`, `--allow-remove-essential`,
`--force-*`, or `--fix-broken` merely to bypass an unexplained transaction.

## Phase 2: install the parallel GA kernel

Disconnect the D435i, save all work, and stop GPU compute jobs. Then execute
the same transaction that was reviewed:

```bash
sudo apt-get install \
  linux-generic \
  linux-modules-nvidia-595-open-generic
```

Read every prompt. Do not reboot if APT or initramfs reports an error.

Do not run these commands after installation:

```text
sudo apt autoremove
sudo apt purge linux-generic-hwe-24.04
sudo apt purge 'linux-image-7.*'
sudo update-alternatives --config x86_64-linux-gnu_gl_conf
```

The current HWE kernel is the rollback path and must remain installed.

## Phase 3: verify installation before reboot

Resolve the newest installed 6.8 GA kernel:

```bash
TARGET_KERNEL="$(
  find /boot -maxdepth 1 -type f -name 'vmlinuz-6.8.*-generic' \
    -printf '%f\n' \
    | sed 's/^vmlinuz-//' \
    | sort -V \
    | tail -n 1
)"
printf 'TARGET_KERNEL=%s\n' "${TARGET_KERNEL}"
```

The value must be nonempty and begin with `6.8.`:

```bash
test -n "${TARGET_KERNEL}"
case "${TARGET_KERNEL}" in
  6.8.*-generic) ;;
  *)
    echo "Unexpected target kernel: ${TARGET_KERNEL}" >&2
    exit 1
    ;;
esac
```

Verify its files and packages:

```bash
test -f "/boot/vmlinuz-${TARGET_KERNEL}"
test -f "/boot/initrd.img-${TARGET_KERNEL}"
test -d "/lib/modules/${TARGET_KERNEL}"

dpkg-query -W -f='${Package} ${Version}\n' \
  "linux-image-${TARGET_KERNEL}" \
  "linux-modules-${TARGET_KERNEL}" \
  "linux-modules-extra-${TARGET_KERNEL}"
```

Verify that a matching NVIDIA module is present:

```bash
modinfo -k "${TARGET_KERNEL}" nvidia \
  | grep -E '^(filename|version|signer|vermagic):'
```

With Secure Boot enabled, `signer` must not be empty. The `vermagic` line must
name the same target kernel.

Rebuild and inspect the GRUB menu:

```bash
sudo update-grub
sudo grep -E "menuentry .*6\.8\..*generic" /boot/grub/grub.cfg
```

Do not manually edit `/boot/grub/grub.cfg`; it is generated content.

Record the installed state:

```bash
{
  date --iso-8601=seconds
  printf 'target_kernel=%s\n' "${TARGET_KERNEL}"
  dpkg-query -W -f='${Package} ${Version}\n' \
    linux-generic \
    linux-modules-nvidia-595-open-generic \
    "linux-image-${TARGET_KERNEL}"
  modinfo -k "${TARGET_KERNEL}" nvidia \
    | grep -E '^(filename|version|signer|vermagic):'
} | tee output/kernel-6.8-migration/installed.txt
```

Only reboot after all checks above pass.

## Phase 4: perform a one-time GRUB boot

Keep the D435i disconnected for this first boot.

```bash
sudo reboot
```

During startup:

1. Repeatedly press `Esc` on a UEFI system. On some legacy BIOS systems, hold
   `Shift`.
2. Select **Advanced options for Ubuntu**.
3. Select **Ubuntu, with Linux 6.8.x-generic**.
4. Do not select recovery mode for the normal test.

This selection affects the current boot. Do not change `GRUB_DEFAULT` yet.

## Phase 5: validate the host after booting 6.8

Before connecting the camera:

```bash
uname -r
```

The result must begin with `6.8.`. If it still reports 7.0, do not run the
camera acceptance test; follow Plan C.

Verify Secure Boot and graphics:

```bash
mokutil --sb-state
nvidia-smi
```

Check for serious boot/module failures:

```bash
systemctl --failed
journalctl -b -p err --no-pager
```

An unrelated historical warning is not automatically fatal, but failures for
`nvidia`, `usb`, `hid_sensor_hub`, display manager, filesystem, or initramfs
must be understood before continuing.

Record the boot:

```bash
cd "${REPO_DIR}"

{
  date --iso-8601=seconds
  uname -a
  mokutil --sb-state
  nvidia-smi
  systemctl --failed
} | tee output/kernel-6.8-migration/boot-6.8.txt
```

## Phase 6: validate the D435i

Connect the D435i directly to the same known-good USB 3 port. Wait ten seconds:

```bash
sleep 10
lsusb | grep -i '8086:0b3a'
rs-enumerate-devices -s
```

Set the serial printed on the exact physical unit. The prompt avoids embedding
a personal device serial in repository documentation:

```bash
read -r -p "D435i serial: " D435I_SERIAL
export D435I_SERIAL

case "${D435I_SERIAL}" in
  ''|*[!0-9]*)
    echo "D435I_SERIAL must contain digits only." >&2
    exit 1
    ;;
esac
```

Run the project hardware gate:

```bash
cd "${REPO_DIR}"

./scripts/preflight_ubuntu.sh \
  --require-camera \
  --serial "${D435I_SERIAL}" \
  --stream-config config/sensors/realsense_streams.yaml \
  |& tee output/kernel-6.8-migration/preflight-6.8.txt
```

The required sampling line must report nonzero stereo, gyro, and accel rates.
The run must end with `PASS` or only understood warnings:

```text
PREFLIGHT_RESULT=PASS
PREFLIGHT_RESULT=PASS_WITH_WARNINGS
```

An installed system librealsense may be reported as informational; the
supported executables must still resolve the patched repository-local library.
Zero-rate IMU output is never an understood warning.

For a longer confirmation:

```bash
./build/linux-release/ovrs_inspect \
  --duration 10 \
  --serial "${D435I_SERIAL}" \
  --stream-config config/sensors/realsense_streams.yaml \
  |& tee output/kernel-6.8-migration/inspect-10s-6.8.txt
```

Continue to README Step 4 only when:

- stereo, gyro, and accelerometer rates are all nonzero;
- malformed frames are zero;
- dropped camera frames are zero;
- rejected timestamps are zero;
- callback errors are zero;
- timestamp monotonic/domain check passes.

This proves the physical capture path on this host. It does not prove VIO
accuracy or calibration quality.

## Success handling

If kernel 6.8 fixes the IMU:

1. Keep kernel 7.0 installed for several successful sessions.
2. Complete at least one ten-second inspector run and one short recorder run.
3. Preserve the reports under `output/kernel-6.8-migration/`.
4. Do not make 6.8 permanent until normal display, suspend/resume, networking,
   USB, and NVIDIA workloads have also been checked.
5. Do not remove older kernels as part of the camera test.

The machine may continue selecting kernel 6.8 manually from GRUB while the
result is evaluated. Permanent GRUB policy is a separate administrator
decision and is deliberately outside this runbook.

## Failure plans

### Plan A: APT simulation is broader than expected

Symptoms include removal of kernel 7.0, GRUB, shim, the desktop, the active
NVIDIA module, or a driver-family transition.

Actions:

1. Do not run the real installation.
2. Save `output/kernel-6.8-migration/apt-simulation.txt`.
3. Capture:

   ```bash
   apt-cache policy \
     linux-generic \
     linux-generic-hwe-24.04 \
     nvidia-driver-595-open \
     linux-modules-nvidia-595-open-generic \
     linux-modules-nvidia-595-open-generic-hwe-24.04
   ```

4. Review held packages:

   ```bash
   apt-mark showhold
   ```

5. Resolve package state explicitly. Do not force the transaction.

### Plan B: package installation or initramfs generation fails

Do not reboot while `dpkg` is incomplete.

Inspect:

```bash
sudo dpkg --audit
sudo journalctl -u apt-daily.service -u apt-daily-upgrade.service \
  --since today --no-pager
df -h / /boot /boot/efi
```

If APT explicitly says configuration was interrupted, use Ubuntu's standard
package recovery:

```bash
sudo dpkg --configure -a
```

Then rerun the Phase 3 verification. Use `sudo apt-get -f install` only after
reviewing its simulation:

```bash
apt-get -s -f install
```

Do not use force flags and do not delete files directly from `/boot`.

### Plan C: kernel 6.8 is installed but absent from GRUB

Boot the normal kernel 7.0 and check:

```bash
find /boot -maxdepth 1 -type f \
  \( -name 'vmlinuz-6.8.*-generic' -o -name 'initrd.img-6.8.*-generic' \) \
  -print
dpkg-query -W 'linux-image-6.8.*-generic'
sudo update-grub
sudo grep -E "menuentry .*6\.8\..*generic" /boot/grub/grub.cfg
```

Do not hand-edit GRUB menu entries. If `update-grub` reports an error, stop and
repair GRUB from the currently working kernel before attempting another boot.

### Plan D: black screen, NVIDIA failure, or kernel 6.8 will not boot

Recovery path:

1. Power-cycle only if the machine is genuinely unresponsive.
2. Open GRUB with `Esc` or `Shift`.
3. Select **Advanced options for Ubuntu**.
4. Boot **Ubuntu, with Linux 7.0.0-28-generic**.
5. Confirm:

   ```bash
   uname -r
   nvidia-smi
   ```

6. Preserve diagnostics:

   ```bash
   journalctl -b -1 -p warning --no-pager \
     | tee output/kernel-6.8-migration/failed-boot.txt
   ```

Do not purge kernel 6.8 immediately. A failed test kernel can remain installed
while its logs are reviewed. Do not disable Secure Boot to hide an unsigned or
missing-module problem.

### Plan E: kernel 6.8 boots correctly but IMU remains at zero

The kernel hypothesis has failed. Do not proceed to Allan capture or VIO.

Perform these checks in order:

1. Unplug the camera for ten seconds and reconnect it directly, without a hub.
2. Try another known-good USB 3 cable and physical port.
3. Run the ten-second `ovrs_inspect` command again.
4. Test accel and gyro in RealSense Viewer on another supported host or
   Windows.
5. If IMU is also absent on another host, treat firmware, calibration-table,
   or Motion Module hardware as the leading fault.

Do not update/downgrade firmware or write an IMU calibration table as a casual
diagnostic. Those operations modify the camera and require a separately
reviewed recovery plan.

Return to kernel 7.0 through GRUB if kernel 6.8 provides no benefit.

### Plan F: preflight passes but later capture has drops

This is a different failure from zero IMU frames.

Check:

- direct USB 3 connection and cable;
- system load and thermal throttling;
- capture rate and queue counters;
- whether another process opened the camera;
- timestamp-domain and monotonicity diagnostics.

Do not conclude that kernel 6.8 fixed estimator drift merely because streams
started. Calibration and estimator acceptance remain separate gates.

## What must never be claimed

- A successful kernel boot is not a successful D435i test.
- A successful D435i stream is not a successful calibration.
- A successful calibration capture is not a successful Kalibr result.
- A successful replay is not proof of real-time estimator accuracy.
- Kernel 6.8 fixing this one host does not prove that every kernel 7.0 host is
  incompatible.

Keep the reports from each gate so those boundaries remain auditable.
