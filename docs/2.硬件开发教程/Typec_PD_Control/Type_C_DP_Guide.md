# TPS65987D + TUSB1146 自定义固件编译与烧录

> *Jennifer Butler 编辑于 2026/1/24*

## 一、硬件说明

### 主要硬件

TPS65987D、TUSB1146

### 硬件定义表

| **TUSB1146 Pins**  |     **Net / Function**     |                           **Note**                           |
| :----------------: | :------------------------: | :----------------------------------------------------------: |
|        EQ0         |             NC             |                                                              |
|        EQ1         |             NC             |                                                              |
|       DPEQ1        |             NC             |                                                              |
|      DPEQ0/A1      |             NC             |                  Set I2C slave address 0x12                  |
|  CAD SNK/DCI DAT   |             NC             |                                                              |
|   HPDIN/DCI CLK    |             NC             |                                                              |
|      SSEQ0/A0      |             NC             |                  Set I2C slave address 0x12                  |
|       SSEQ1        |             NC             |                                                              |
| **TPS65987D Pins** |     **Net / Function**     |                           **Note**                           |
|      I2C3_SCL      |          TUSB_SCL          |                   Connect to TUSB1146 SCL                    |
|      I2C3_SDA      |          TUSB_SDA          |                   Connect to TUSB1146 SDA                    |
|      SPI_MISO      |         FLASH_MISO         |                                                              |
|      SPI_MOSI      |         FLASH_MOSI         |                                                              |
|      SPI_CLK       |         FLASH_CLK          |                                                              |
|       SPI_SS       |          FLASH_CS          |                                                              |
|        HPD         |        DP_HDP0_3.3V        |                                                              |
|       GPIO1        | Connect to GND throught 1M |                                                              |
|      C_USB_N       |           USB_N            |                                                              |
|      C_USB_P       |           USB_P            |                                                              |
|       C_CC1        |            CC1             |                                                              |
|       C_CC2        |            CC2             |                                                              |
|      I2C1_SCL      |         TPS_DE_SCL         |                          for Debug                           |
|      I2C1_SDA      |         TPS_DE_SDA         |                          for Debug                           |
|      I2C2_SCL      |                            |                 Connect to VCC throught 10K                  |
|      I2C_SDA       |                            |                 Connect to VCC throught 10K                  |
|       HEESET       |                            |                            HREST                             |
|       ADCIN1       |                            | Connect to GND, Config as Safe Configuration and BP_NoResponse |
|       ADCIN2       |                            |        Connect to GND, Config Debug I2C address 0x20         |

### 原理图

## 二、步骤

### (一)环境准备

1. **下载并安装 [TPS6598X-CONFIG](https://www.ti.com/tool/TPS6598X-CONFIG)**
2. **通过 FT232H 模块和 4Pin 1.5mm 间距 PCB 测试夹连接底板与电脑**

### (二)创建硬件配置文件

1. 点击`Project->Now Project`创建新工程

2. 器件选择`TPS65987DDK`

3. 类型选择`Dual Role Port (DRP), prefers power source`

4. 切换到`Device1`选项卡

5. 进入`USB Configuration `，勾选`Host Present,DP Supported,USB3 Dual Data Role`

6. 进入`Global System Configuration`，修改`TBT Controller Type`为`Default`

7. 进入`Port Configuration`，修改`Port Configuration`为`DRP`，修改`Type-C Supported Options`为`Try.SRC`，勾选`Set UVP to 4.5V`

8. 进入`Port Control`，勾选`Unconstrained Power, Automatic Sink Cap`

9. 进入`Transmit Source Capabilities`，`Number of Bank O Source PDOs`填`1`，勾选`USB Capable,USB Suspend Supported`，取消勾选`Unchunked Extended Msg Supported`

10. 进入`Transmit Sink Capabilities`，`Number of Sink PDOs`填`1`，`Operating Current,Maximum Operating Current,Minimum Operating Current`均填`0.01A`，取消勾选`Ask For Max,Higher Capability`

11. 进入`Autonegotiate Sink`，取消勾选`Autonegotiate Variable Sink Enable,Autonegotiate Battery Sink Enable,Auto Compute Sink Min Power`，勾选`No USB Suspend`，修改`Operating Current,Min Operating Current,Maximum Current`为`0.01A`，修改`Auto Negotiate Sink Min Required Power`为`0W`

12. 进入`Alternate Mode Entry Queue`，修改`Alternate Mode Entry Queue record #1`的`SVID`为`0xff01`，修改`Alternate Mode Entry Queue record #2`的`SVID`为`0x0`

13. 进入`PD3 Configuration Register`，取消勾选`Unchunked Messaging Supported`，勾选`Support Source Cap Extended Message,Support Status Message,Support Sink Cap Extended,Support Get Revision`

14. 进入`Delay Configuration`，修改`HPD Delay`为`50ms`

15. 进入`Tx Identity`，修改`Vendor ID`为`0x451`，修改`Product Type DFP`为`Undefined`，修改`USB Highest Speed`为`USB3.2Gen2`，`Device Capable,Host Capable`取消勾选`USB4`

16. 进入`Display Port Capabilities`，修改`DP Port Caapability`为`DP DFP_D only`，修改`Preferred DP Role`为`Prefers DFP_D`，修改`DFP_D/UFP_D Connected`为`DFP_D / UFP_D Connected`，勾选`Supports USB Gen 2 signalling,DFPD Receptacle or UFPD Plug Pin Assignment C D E`，取消勾选`UFPD Receptacle or DFPD Plug Pin Assignment C D`

17. 进入`Intel VID Config Register`，取消勾选`Enable Intel VID,Enable Intel Thunderbolt Mode,Data Status HPD Events,TBT Mode Autoentry`

18. 进入`I/O Config`，`GPIO #0`的`Mapped Event`修改为`Port 1 UFP DFP Event`，`GPIO Polarity`修改为`Inverted Event`，勾选`Open Drain Output Enable`，`GPIO #03`的`Multiplexing for GPIO 3 pin`修改为`Pin Multiplexed to Alternate Function (DP HPD Port 1)`，`GPIO #4,GPIO #7-#21`的`Multiplexed`均修改为`Pin Multiplexed to GPIO`，`Mapped Event`均修改为`Disable`

19. 进入`App Config Binary Data Indices`，`Port 1 I2C Record Number of Indices`修改为`6`

20. 进入`I2C Master Configuration`，`Slave 1 I2C Address`修改为`0x12`

21. 进入`Transmit Sink Capabilities Extended Data Block (SCEDB) Register`，勾选`Can Tolerate VBUS Voltage Drop,VBUS Powered,Mains Powered`

22. 进入`Tx Source Capabilities Extended Data Block`，勾选`Ground Pin Supported,External Supply Present,External Supply is Unconstrained`

> *注意，没有提到的参数保持工程默认配置，不要乱改，不要乱改，不要乱改！*

### (三)编写 I2C 控制事件

1. 勾选菜单栏`Settings->Show I2C Controller Events Table`

2. 将现有所有事件的`Trigger Event`修改为`NULL`，`Data Lenght,Slave Address Index`修改为`0`，`Data`修改为`0x0`

3. 按照以下表格进行修改

   | **Record Index** |          Trigger Event           | **Data Length** | **Slave Address Index** |         **Data **         |                             Note                             |
   | :--------------: | :------------------------------: | :-------------: | :---------------------: | :-----------------------: | :----------------------------------------------------------: |
   |        1         |       Cable Attach CC_1 PD       |        2        |            1            |       0x0A 11 1C F3       |      USB正插模式，无翻转，4 Lane USB 3.2，自动增益开启       |
   |        2         |       Cable Attach CC_2 PD       |        2        |            1            |       0x0A 15 1C F3       |       USB反插模式，翻转，4 Lane USB 3.2，自动增益开启        |
   |        3         | Displayport Pin Config C CC_1 PD |        2        |            1            |    0x0A 12 10 88 11 88    |        DP正插模式,无翻转，4 Lane DP1.4，固定增益开启         |
   |        4         | Displayport Pin Config C CC_2 PD |        2        |            1            |    0x0A 16 10 88 11 88    |         DP反插模式，翻转，4 Lane DP1.4，固定增益开启         |
   |        5         | Displayport Pin Config D CC_1 PD |        2        |            1            | 0x0A 13 10 88 11 88 1C F3 | 全功能正插模式，无翻转，2 Lane DP1.4 +  2 Lane USB3.2，双增益开启 |
   |        6         | Displayport Pin Config D CC_2 PD |        2        |            1            | 0x0A 17 10 88 11 88 1C F3 | 全功能反插模式，翻转，2 Lane DP1.4 + 2 Lane USB3.2，双增益开启 |

### (四)生成固件

菜单栏点击`Binary->Save Binary`，选择`Low Region (Full Header)`，点击`Save`

## 三、烧录固件

  - 通过 FT232H 模块和 4Pin 1.5mm 间距 PCB 测试夹连接底板与电脑

  - 菜单栏点击`Binary->Flash From Binary File`，选择生成的固件文件，点击烧录

