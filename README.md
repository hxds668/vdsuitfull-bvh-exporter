# VDSuitFull Linux BVH Exporter

This is a Linux x64/arm64 command-line BVH recorder for the VDSuitFull Linux SDK.

## What It Does

- Loads the SDK `.so` with `dlopen` / `dlsym`.
- Uses `_MocapDataWithVirtual_` callback data.
- Exports Studio-style FullHands BVH:
  - 61 BVH nodes.
  - 186 motion columns.
  - Body, head, both hands and fingers.
- Optionally exports BodyOnly BVH.
- Supports A-pose and P-pose calibration from the interactive menu.
- Prepends a Studio-style rest frame by default:
  - Hips position `0 111 0`.
  - All rotations `0`.

## Expected Layout

Recommended location inside the Linux SDK folder:

```text
Full_Release_LinuxSDK/
  vdsuitfull-sdktest/
  vdsuitfull-bvh-exporter/
    build.sh
    include/
    src/
    bin/
```

The program will look for the SDK library in these common relative locations:

```text
./lib/arm64/libVDMocapSDK_miniArm64.so
../vdsuitfull-sdktest/lib/arm64/libVDMocapSDK_miniArm64.so
../vdsuitfull-sdktest/lib/ubuntu22.04_arm64/libVDMocapSDK_VDSuitMiniArm64.so
../vdsuitfull-sdktest/lib/ubuntu20.04_arm64/libVDMocapSDK_VDSuitMiniArm64.so
```

You can always override it:

```bash
./bin/arm64/vdsuit_bvh_exporter --lib /path/to/libVDMocapSDK_miniArm64.so
```

## Build

On ARM64 Linux:

```bash
chmod +x build.sh
./build.sh
```

The output binary will be:

```text
./bin/arm64/vdsuit_bvh_exporter
```

On x64 Linux, it builds:

```text
./bin/x64/vdsuit_bvh_exporter
```

## Run

Interactive mode:

```bash
./bin/arm64/vdsuit_bvh_exporter
```

Interactive menu shortcuts:

```text
1  Connect
2  Disconnect
3  Start/stop BVH recording
4  Cancel recording
5  Toggle FullHands/BodyOnly
6  Toggle Studio rest frame
7  Toggle data print
8  Show gesture
9  A-pose calibration
P  P-pose calibration
0  Exit
```

During calibration, press `Q` to cancel without pressing Enter.

Auto-record for 10 seconds:

```bash
./bin/arm64/vdsuit_bvh_exporter --duration 10
```

Useful options:

```bash
--out DIR
--freq 60
--freq 72
--freq 80
--freq 96
--body-only
--no-rest-frame
--lib PATH
```

## ARM Linux Notes

If connection fails, check:

- The VDSuit USB device is connected.
- The user has serial permission, usually:

```bash
sudo usermod -aG dialout "$USER"
```

- Log out and back in after changing groups.
- If the Exar/MxL USB serial chip is used, install the provided driver:

```bash
cd ../vdsuitfull-sdktest/driver/Linux_3.6
make
sudo insmod ./xr_usb_serial_common.ko
```

Then check device nodes:

```bash
ls /dev/ttyXRUSB*
```
