# Ubuntu 22.04.5 运行说明

拷贝整个 `vdsuitfull-bvh-exporter` 文件夹到 Ubuntu 22.04.5。

## ARM64 设备

```bash
cd vdsuitfull-bvh-exporter
chmod +x build.sh
./build.sh
./bin/arm64/vdsuit_bvh_exporter
```

程序默认会找：

```text
./lib/arm64/libVDMocapSDK_miniArm64.so
```

## x86_64 电脑

```bash
cd vdsuitfull-bvh-exporter
chmod +x build.sh
./build.sh
./bin/x64/vdsuit_bvh_exporter
```

程序默认会找：

```text
./lib/x64/libVDMocapSDK_mini.so
```

## 如果还是提示找不到 so

可以手动指定：

```bash
./bin/arm64/vdsuit_bvh_exporter --lib ./lib/arm64/libVDMocapSDK_miniArm64.so
```

## 如果能加载 so 但连不上设备

检查 USB/串口权限：

```bash
ls /dev/ttyUSB* /dev/ttyACM*
sudo usermod -aG dialout $USER
```

然后重新登录。必要时参考 `driver/Linux_3.6` 里的官方驱动文件。

## XmlCQHand / XmlCQ* 未定义符号说明

如果 Ubuntu 22.04.5 上提示 `XmlCQHand` / `XmlCQ*` 相关未定义符号，通常不是业务代码问题，而是厂商 SDK `.so` 里包含了可选的手势/XML辅助代码。新版工程已经把 `dlopen` 改为 `RTLD_LAZY`，避免启动时强制解析这些暂时不用的符号。

建议优先运行：

```bash
./bin/arm64/vdsuit_bvh_exporter --lib ./lib/arm64/libVDMocapSDK_miniArm64.so
```

如果仍失败，再试：

```bash
./bin/arm64/vdsuit_bvh_exporter --lib ./lib/arm64/libVDMocapSDK_VDSuitMiniArm64.so
```

如果只在选择 `8. Show gesture` 时崩溃/报符号问题，先不要使用手势菜单；BVH录制、连接和标定可先独立验证。

## XR21x USB 串口驱动（商家 V1G，Kernel 3.6 到 6.8）

商家给的新驱动已放在：

```text
driver/XR21x_Linux3.6_newer_V1G/
```

这个驱动用于让接收器在 Linux 下识别成 `/dev/ttyXRUSB0` 这类串口设备。它解决的是“设备连接/串口识别”问题，不解决 `.so` 自身的 `XmlCQHand` 未定义符号问题。

Ubuntu 22.04.5 上安装：

```bash
cd vdsuitfull-bvh-exporter/driver/XR21x_Linux3.6_newer_V1G
sudo apt update
sudo apt install -y build-essential linux-headers-$(uname -r)
make
sudo insmod ./xr_usb_serial_common.ko
```

插入接收器后检查：

```bash
lsusb
dmesg | tail -50
ls /dev/ttyXRUSB* /dev/ttyUSB* /dev/ttyACM*
```

如果系统先加载了 `cdc_acm` 或 `usbserial`，可以按商家 README 里的方式切换：

```bash
sudo rmmod cdc_acm
sudo modprobe -r usbserial
sudo modprobe usbserial
sudo insmod ./xr_usb_serial_common.ko
```

如果要开机自动加载：

```bash
sudo make modules_install
sudo depmod -a
sudo modprobe xr_usb_serial_common
```

普通用户访问串口权限：

```bash
sudo usermod -aG dialout $USER
```

执行后需要重新登录。
