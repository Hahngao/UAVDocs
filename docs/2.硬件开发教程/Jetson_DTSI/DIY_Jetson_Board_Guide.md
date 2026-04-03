# Jeston Orin Nano/NX 自定义底板对应设备树编写与系统编译烧录

> *Jennifer Butler 编辑于 2026/1/25
## 一、硬件说明

### 主要硬件

Jetson Orin Nano/NX

### 部分引脚定义表

|    Pins     |     Net     |                            Note                             |
| :---------: | :---------: | :---------------------------------------------------------: |
|   DP0_HPD   |   DP_HDP0   |                         DP HDP Port                         |
|    GPIO0    |   VBUS_EN   |                    Use as usb2.0 id pin                     |
|    GPIO1    |     NC      |                                                             |
|    GPIO2    |   BT_WAKE   |                 Bluetooth wake up  control                  |
|    GPIO3    |    BT_EN    |                  Bluetooth enable  control                  |
|    GPIO4    |    LED_R    |               Connect to LED red,  active low               |
|    GPIO5    |   WiFi_EN   |                     WiFi enable control                     |
|    GPIO6    | CAMMUX_SEL  | Connect to  TS3USB221EDRCR to control mux CAM1 and CAM2 I2C |
|    GPIO7    |    LED_G    |                    LED green, active low                    |
|    GPIO8    |  FAN_TACH   |                     Use to get fan  rpm                     |
|    GPIO9    |   OLED_EN   |                On board OLED enable  control                |
|   GPIO10    |  M2E_ALERT  |                 Connect to M.2 E key  ALERT                 |
|   GPIO11    | SPI0_DRDY2  |              Connect to on board  BMI088 INT3               |
|   GPIO12    | SPI0_DRDY1  |              Connect to on board  BMI088 INT1               |
|   GPIO13    |    LED_B    |                    LED blue, active low                     |
|   GPIO14    |   FAN_PWM   |                         Fan control                         |
| CAM_I2C_SCL | CAM_I2C_SCL |       Connect to  TS3USB221EDRCR for CAM1 an CAM2 SCL       |
| CAM_I2C_SDA | CAM_I2C_SDA |       Connect to  TS3USB221EDRCR for CAM1 an CAM2 SDA       |
|  SPI0_SCK   |  SPI0_SCK   |              Connect to on board  BMI088 SCLK               |
|  SPI0_MISO  |  SPI0_MISO  |              Connect to on board  BMI088 MISO               |
|  SPI0_MOSI  |  SPI0_MOSI  |              Connect to on board  BMI088 MOSI               |
|  SPI0_CS0   |  SPI0_CS0   |               Connect to on board  BMI088 CS1               |
|  SPI0_CS1   |  SPI0_CS1   |               Connect to on board  BMI088 CS2               |
| SDMMC_DAT0  | SDMMC_DAT0  |               Connect to M.2 E key  SDMMC D0                |
| SDMMC_DAT1  |  M2M_ALERT  |                 Connect to M.2 M key  ALERT                 |
| SDMMC_DAT2  | SDMMC_DAT2  |               Connect to M.2 E key  SDMMC D2                |
| SDMMC_DAT3  | SDMMC_DAT3  |               Connect to M.2 E key  SDMMC D3                |
|  SDMMC_CMD  |  SDMMC_CMD  |               Connect to M.2 E key  SDMMC CMD               |
|  SDMMC_CLK  |  SDMMC_CLK  |              Connect to M.2 E key  SDMMC SCLK               |
|  I2C0_SCL   |     NC      |                                                             |
|  I2C0_SDA   |     NC      |                                                             |
|  I2C1_SCL   |  I2C1_SCL   |                Connect to on board  OLED_SCL                |
|  I2C1_SDA   |  I2C1_SDA   |                Connect to on board  OLED SDA                |
|  I2C2_SCL   |  I2C2_SCL   |           Connect to M.2 E key  and M.2 M Key SCL           |
|  I2C2_SDA   |  I2C2_SDA   |           Connect to M.2 E key  and M.2 M Key SDA           |
|   CAN1_RX   |   CAN1_RX   |                    Connect to TJA1051 RX                    |
|   CAN1_TX   |   CAN1_TX   |                    Connect to TJA1051 TX                    |
|  UART0_TXD  |  UART0_TXD  |                  Connect to M.2 E Key  TXD                  |
|  UART0_RXD  |  UART0_RXD  |                  Connect to M.2 E Key  RXD                  |
|  UART0_RTS  |  UART0_RTS  |                  Connect to M.2 E Key  RTS                  |
|  UART0_CTS  |  UART0_CTS  |                  Connect to M.2 E Key  CTS                  |
|  UART1_TXD  |  UART1_TXD  |                   Connect to exp APM  RXD                   |
|  UART1_RXD  |  UART1_RXD  |                   Connect to exp APM  TXD                   |
|  UART1_RTS  |  UART1_RTS  |                   Connect to exp APM  CTS                   |
|  UART1_CTS  |  UART1_CTS  |                   Connect to exp APM  RTS                   |
|  UART2_TXD  |   DE_TXD    |                        Use for Debug                        |
|  UART2_RXD  |   DE_RXD    |                        Use for Debug                        |
|  USB0_D_P   |    USB_P    |                    Connect to Type-C DP                     |
|  USB0_D_N   |    USB_N    |                    Connect to Type-C DM                     |
|  USB1_D_P   |     NC      |                                                             |
|  USB1_D_N   |     NC      |                                                             |
|  USB2_D_P   |   USB2_P    |                  Connect to M.2 E Key  DP                   |
|  USB2_D_N   |   USB2_N    |                  Connect to M.2 E Key  DM                   |
| USBSS_TX_P  | USBSS_TX_P  |                  Connect to TUSB1146  RX P                  |
| USBSS_TX_N  | USBSS_TX_N  |                  Connect to TUSB1146  RX N                  |
| USBSS_RX_P  | USBSS_RX_P  |                  Connect to TUSB1146  TX P                  |
| USBSS_RX_N  | USBSS_RX_N  |                  Connect to TUSB1146  TX N                  |
|  CSI0_D0_P  |  CSI0_D0_P  |                 Connect to CAM1  connector                  |
|  CSI0_D0_N  |  CSI0_D0_N  |                 Connect to CAM1  connector                  |
|  CSI0_D1_P  |  CSI0_D1_P  |                 Connect to CAM1  connector                  |
|  CSI0_D1_N  |  CSI0_D1_N  |                 Connect to CAM1  connector                  |
| CSI0_CLK_P  | CSI0_CLK_P  |                 Connect to CAM1  connector                  |
| CSI0_CLK_N  | CSI0_CLK_N  |                 Connect to CAM1  connector                  |
|  CSI1_D0_P  |  CSI1_D0_P  |                 Connect to CAM1  connector                  |
|  CSI1_D0_N  |  CSI1_D0_N  |                 Connect to CAM1  connector                  |
|  CSI1_D1_P  |  CSI1_D1_P  |                 Connect to CAM1  connector                  |
|  CSI1_D1_N  |  CSI1_D1_N  |                 Connect to CAM1  connector                  |
| CSI1_CLK_P  |     NC      |                                                             |
| CSI1_CLK_N  |     NC      |                                                             |
|  CSI2_D0_P  |  CSI2_D0_P  |                 Connect to CAM2  connector                  |
|  CSI2_D0_N  |  CSI2_D0_N  |                 Connect to CAM2  connector                  |
|  CSI2_D1_P  |  CSI2_D1_P  |                 Connect to CAM2  connector                  |
|  CSI2_D1_N  |  CSI2_D1_N  |                 Connect to CAM2  connector                  |
| CSI2_CLK_P  | CSI2_CLK_P  |                 Connect to CAM2  connector                  |
| CSI2_CLK_N  | CSI2_CLK_N  |                 Connect to CAM2  connector                  |
|  CSI3_D0_P  |  CSI3_D0_P  |                 Connect to CAM2  connector                  |
|  CSI3_D0_N  |  CSI3_D0_N  |                 Connect to CAM2  connector                  |
|  CSI3_D1_P  |  CSI3_D1_P  |                 Connect to CAM2  connector                  |
|  CSI3_D1_N  |  CSI3_D1_N  |                 Connect to CAM2  connector                  |
| CSI3_CLK_P  |     NC      |                                                             |
| CSI3_CLK_N  |     NC      |                                                             |

## 二、步骤

### （一）配置自定义设备树

1.  **下载配置表**：从 [英伟达下载中心](https://developer.nvidia.cn/embedded/downloads) 下载 `Jetson_Nano_DeveloperKit_Users_Pinmux_Configuration.xlsm`
2.  **编辑配置**：
    *   使用 Excel 打开文件 (需启用宏)
    *   切换到底部的 `Jetson Orin Nano&NX Pinmux DP` 工作表
    *   根据上面的表格，逐行修改对应引脚的 Customer Usage
3.  **生成文件**：
    *   点击表格右上角的 `Generate DT File` 按钮。
    *   保存生成的三个 `.dtsi` 文件 (包含 `pinmux`, `gpio`, `padvoltage`)

### （二）获取官方系统源码

访问 [Jetson Linux R36.4.4 下载页](https://developer.nvidia.com/embedded/jetson-linux-r3644) 下载

| 文件名 | 描述 | 用途 |
| :--- | :--- | :--- |
| `Jetson_Linux_r36.4.4_aarch64.tbz2` | Driver Package (BSP) | 刷机工具和基础文件系统 |
| `Tegra_Linux_Sample-Root-Filesystem_...tbz2` | Sample Rootfs | Ubuntu 根文件系统模板 |
| `public_sources.tbz2` | BSP Sources | 内核和引导程序源码 |
| `aarch64--glibc--stable-2022.08-1.tar.bz2` | Bootlin Toolchain | 交叉编译工具链 |

*注：工具链请在页面中的 "Toolchain" 部分寻找 GCC 11.3 版本*

### （三）搭建编译环境

#### 1. 创建工作目录并解压
将所有文件放在 `~/Jetson` 目录下

```bash
mkdir -p ~/Jetson
cd ~/Jetson

tar xf Jetson_Linux_r36.4.4_aarch64.tbz2

cd Linux_for_Tegra/rootfs/
sudo tar xpf ../../Tegra_Linux_Sample-Root-Filesystem_R36.4.4_aarch64.tbz2
cd ../..

tar xf aarch64--glibc--stable-2022.08-1.tar.bz2

tar xf public_sources.tbz2
cd Linux_for_Tegra/source

tar xf kernel_src.tbz2
tar xf kernel_oot_modules_src.tbz2
tar xf nvidia_kernel_display_driver_source.tbz2
cd ../.. 
```

#### 2. 安装系统依赖库

```bash
sudo apt update
sudo apt install -y build-essential bc flex bison libssl-dev zstd qemu-user-static \
libxml2-utils cpio dosfstools mtools xmlstarlet sshpass abootimg git python3-pip \
device-tree-compiler
```

#### 3. 配置临时环境变量

```bash
export CROSS_COMPILE=$HOME/Jetson/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
export ARCH=arm64

export KERNEL_OUT=$HOME/Jetson/Linux_for_Tegra/source/kernel_out

export INSTALL_MOD_PATH=$HOME/Jetson/Linux_for_Tegra/rootfs
```

### （四）集成自定义设备树

#### 1. 放置生成的配置文件
将步骤（一）中 Excel 生成的 3 个 `.dtsi` 文件复制到以下路径：
`~/Jetson/Linux_for_Tegra/source/hardware/nvidia/t23x/nv-public/nv-platform/`

#### 2. 修改顶层设备树源码
通过修改主设备树文件来包含自定义配置
目标文件：`~/Jetson/Linux_for_Tegra/source/hardware/nvidia/t23x/nv-public/nv-platform/tegra234-p3768-0000+p3767-0000.dts`

该在文件末尾添加如下内容

```ini
#include "pinmux文件名.dtsi"
#include "gpio文件名.dtsi"
#include "padvoltage文件名.dtsi"

/* BMI088 (SPI0) */
/* SPI0_CS0接BMI088_CS1(Accel), SPI0_CS1接BMI088_CS2(Gyro) */
/* spi@3210000 (spi0) */
&spi0 {
    status = "okay";
    
    /* BMI088 加速度计节点 (CS0) */
    bmi088_accel@0 {
        compatible = "bosch,bmi088_accel_spi"; 
        reg = <0>; 
        spi-max-frequency = <10000000>;
        interrupt-parent = <&tegra_main_gpio>;
        /* GPIO12对应 PN.01 (TEGRA234_MAIN_GPIO(N, 1)) */
        interrupts = <TEGRA234_MAIN_GPIO(N, 1) IRQ_TYPE_EDGE_RISING>; 
    };

    /* BMI088 陀螺仪节点 (CS1) */
    bmi088_gyro@1 {
        compatible = "bosch,bmi088_gyro_spi";
        reg = <1>; 
        spi-max-frequency = <10000000>;
        interrupt-parent = <&tegra_main_gpio>;
        /* GPIO11对应 PQ.06 (TEGRA234_MAIN_GPIO(Q, 6)) */
        interrupts = <TEGRA234_MAIN_GPIO(Q, 6) IRQ_TYPE_EDGE_RISING>; 
    };
};
```

### （五）配置系统内核 (开启驱动)

手动开启开启 WiFi 和传感器驱动

1.  **创建输出目录**
    ```bash
    mkdir -p $KERNEL_OUT
    ```

2.  **加载默认配置**
    ```bash
    cd ~/Jetson/Linux_for_Tegra/source/kernel/kernel-jammy-src
    make O=$KERNEL_OUT tegra_defconfig
    ```

3.  **进入图形化配置界面**
    ```bash
    make O=$KERNEL_OUT menuconfig
    ```
    开启如下项：
    
    *   `Networking support -> CAN bus subsystem support` (CAN)
    *   `Device Drivers -> Network device support -> Wireless LAN -> Intel Wireless WiFi ...` (AX210/AX200网卡)
    *   `Device Drivers -> Industrial I/O support -> Inertial measurement units -> BMI088...` (IMU)

### （六）编译全套系统

1.  **编译内核镜像和设备树**
    
    ```bash
    make -j$(nproc) O=$KERNEL_OUT Image dtbs
    ```

2.  **编译内核模块**
    
    ```bash
    make -j$(nproc) O=$KERNEL_OUT modules
    ```
    
3.  **安装模块到根文件系统**
    
    ```bash
    sudo -E make O=$KERNEL_OUT modules_install
    ```

### （七）部署产物与烧录

#### 1. 替换内核与设备树

```bash
cp $KERNEL_OUT/arch/arm64/boot/Image ~/Jetson/Linux_for_Tegra/kernel/Image

cp $KERNEL_OUT/arch/arm64/boot/dts/nvidia/tegra234-p3768-0000+p3767-*.dtb ~/Jetson/Linux_for_Tegra/kernel/dtb/
cp $KERNEL_OUT/arch/arm64/boot/dts/nvidia/tegra234-p3768-0000+p3767-*.dtbo ~/Jetson/Linux_for_Tegra/kernel/dtb/
```

#### 2. 初始化根文件系统并安装 Nvidia 闭源驱动

```bash
cd ~/Jetson/Linux_for_Tegra
sudo ./apply_binaries.sh
```

#### 3. 烧录到 NVMe SSD

**第一步：生成烧录镜像**

```bash
sudo ./tools/kernel_flash/l4t_initrd_flash.sh \
  --no-flash \
  --external-device nvme0n1p1 \
  -c tools/kernel_flash/flash_l4t_t234_nvme.xml \
  -p "-c bootloader/generic/cfg/flash_t234_qspi.xml" \
  --showlogs \
  --network usb0 \
  jetson-orin-nano-devkit internal
```
*当看到 `Finish generating flash package` 和 `Success` 时表示镜像生成成功*

**第二步：进入 Recovery 模式并烧录**
1.  拔掉载板板电源
2.  按住`Recovery`键
3.  插上电源
4.  松开`Recovery`键
5.  用 USB Type-C 数据线连接载板和电脑
6.  在电脑终端输入 `lsusb`，如果看到 `NVIDIA Corp.` 字样，说明连接成功
7.  执行烧录命令

```bash
sudo ./tools/kernel_flash/l4t_initrd_flash.sh \
  --flash-only \
  --external-device nvme0n1p1 \
  -c tools/kernel_flash/flash_l4t_t234_nvme.xml \
  -p "-c bootloader/generic/cfg/flash_t234_qspi.xml" \
  --showlogs \
  --network usb0 \
  jetson-orin-nano-devkit internal
```

等待烧录进度条走完，载板将自动重启，进入 Ubuntu 的配置界面
