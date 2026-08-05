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
- Optionally streams the 23 body joints to a UDP endpoint as JSON datagrams.
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
S  Start/stop UDP forwarding
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
--send 192.168.1.100:9000
--body-only
--no-rest-frame
--lib PATH
```

## UDP Pose Streaming

Start a UDP listener on the receiving machine, then pass its IPv4 address and port:

```bash
./bin/arm64/vdsuit_bvh_exporter --send 192.168.1.100:9000
```

`--send` only configures the destination. In interactive mode, connect the suit with menu option `1`, then press `S` to start forwarding; press `S` again to stop. No UDP data is sent merely because the device connected. Disconnecting the suit also stops forwarding, so a later reconnect requires pressing `S` again. Auto-record mode does not start forwarding implicitly.

Forwarding is independent of BVH recording. The sender keeps only four pending frames so a slow network cannot block the SDK callback.

The wire format is compact UTF-8 JSON, with one complete message in each UDP datagram. When a forwarding session starts, it sends one `skeleton` datagram containing the 23 body joint names, parent indices, initial positions, and parent-relative offsets. Each following `frame` datagram contains `frame_index` and 23 position/quaternion pairs in the same joint order. Datagram payloads retain a trailing newline for compatibility with line-oriented JSON tools:

```json
{"type":"skeleton","version":1,"joint_count":23,"coordinate_system":"WS_Geo","position_unit":"m","quaternion_order":"wxyz","joints":[{"index":0,"name":"Hips","parent_index":-1,"initial_position":[0,0,1.11],"offset":[0,0,1.11]}]}
{"type":"frame","version":1,"frame_index":123,"joints":[{"position":[0,0,1.11],"quaternion":[1,0,0,0]}]}
```

The examples above abbreviate the `joints` arrays. Actual messages always contain 23 body joints; hand and finger arrays are not sent. Output indices and names retain `_BodyNodes_` order, but left/right limb data sources are intentionally exchanged: output right leg indices `1–4` read SDK left leg `5–8`, output left leg `5–8` read SDK right leg `1–4`, output right arm `15–18` reads SDK left arm `19–22`, and output left arm `19–22` reads SDK right arm `15–18`. Center joints `0` and `9–14` are unchanged. FullHands BVH also exchanges the left/right finger data sources.

The JSON schema remains unchanged. Positions stay in SDK `WS_Geo` and use meters. Each `quaternion` remains an absolute global rotation in `w,x,y,z` order. After the left/right source exchange, rotations are mirrored across the lateral X axis as `(w,x,y,z) -> (w,x,-y,-z)`, allowing the player's existing global-to-local conversion to preserve the absolute non-T-pose arm orientation while reversing the left/right rotation basis. Invalid quaternion frames are dropped. Because UDP does not guarantee delivery, start the receiver before the exporter if it needs the one-time skeleton datagram.

Run the network protocol tests with:

```bash
bash tests/run_tests.sh
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
