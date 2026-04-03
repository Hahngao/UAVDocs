# AM32 CAN ESC自定义固件编译与烧录

> *Jennifer Butler 编辑于 2026/1/23*

## 一、硬件说明

### 主要硬件

STSPIN32G4TR、INA240A2、WS2812、TJA1462ATK

### 引脚定义表

| **MCU Pins** | **Net / Function** |                     **Note**                      |
| :----------: | :----------------: | :-----------------------------------------------: |
|     PA0      |      Phase C       |                                                   |
|     PA1      |        VNP         |                                                   |
|     PA2      |       DSHORT       |                       RC IN                       |
|     PA3      |        VNP         |                                                   |
|     PA4      |      Phase B       |                                                   |
|     PA5      |      Phase A       |                                                   |
|     PA6      |   Voltage_Sensor   |            Connect to 1K/(10K+1K) Net             |
|     PA7      |   Current_Sensor   |          Connect to INA240A2 With 0.5mR           |
|     PA11     |       CAN_RX       |                                                   |
|     PA12     |       CAN_TX       |                                                   |
|     PA13     |       SWDIO        |                                                   |
|     PA14     |       SWCLK        |                                                   |
|     PB1      | Temperature_Sensor |        Connect to 10K/(10K NTC B=3950) Net        |
|     PB8      |     WS2813_DI      |                  WS2812 Control                   |
|     PB10     |        RPM         |              For Debug, Not Connect               |
|     PC8      |      I2C3_SCL      | Internal Connect to Gate Driver (Not Use by AM32) |
|     PC9      |      I2C3_SDA      | Internal Connect to Gate Driver (Not Use by AM32) |
|     PE7      |        WAKE        | Internal Connect to Gate Driver (Not Use by AM32) |
|     PE8      |     TIM1_CH1N      |       Internal Connect to Gate Driver (AL)        |
|     PE9      |      TIM1_CH1      |       Internal Connect to Gate Driver (AH)        |
|     PE10     |     TIM1_CH2N      |       Internal Connect to Gate Driver (BL)        |
|     PE11     |      TIM1_CH2      |       Internal Connect to Gate Driver (BH)        |
|     PE12     |     TIM1_CH3N      |       Internal Connect to Gate Driver (CL)        |
|     PE13     |      TIM1_CH3      |       Internal Connect to Gate Driver (CH)        |
|     PE14     |       READY        | Internal Connect to Gate Driver (Not Use by AM32) |
|     PE15     |       NFAULT       | Internal Connect to Gate Driver (Not Use by AM32) |
|     PF0      |       OSC IN       |              Connect to 8MHz Crystal              |
|     PF1      |      OSC OUT       |              Connect to 8MHz Crystal              |
|     PG10     |        NRST        |                     Reset Pin                     |

### 原理图

![image](./image.png)

## 二、步骤

### (一) 编译 AM32 G431 Boot loader

1. **安装 [VS Code](https://code.visualstudio.com/)**

2. **安装 C/C++ tools Cortex-Debug Makefile Tools 插件**

3. **安装 [Git](https://git-scm.com/)** 

4. **获取源码**

   - 选择一个工作文件夹，并在工作文件夹中打开 VS Code

   -   打开终端 (`Ctrl + ~`)

   -   输入以下命令拉取 ExpressLRS 源码

       ```powershell
       git clone https://github.com/am32-firmware/AM32-bootloader.git
       ```
       
       注意：如果已经有源码，直接在 VS Code 中 `File -> Open Folder` 打开 `AM32-bootloader` 文件夹即可
       
   -  切换到 master 分支
   
5. **安装必要依赖**

   - 打开终端 (`Ctrl + ~`)

   -   输入以下命令安装依赖

       ```powershell
       cd .\env_setup_scripts\
       .\gcc_windows_env_setup.cmd
       ```

   -   等待终端输出`Script completed successfully.`依赖安装成功

6. **回到根目录，配置临时环境变量并开始编译**

   - 打开终端 (`Ctrl + ~`)

   - 输入以下命令

     ```powershell
     cd ..
     $env:PATH = "$PWD\tools\windows\make\bin;$PWD\tools\windows\xpack-arm-none-eabi-gcc-10.3.1-2.3\bin;" + $env:PATH
     make AM32_G431_BOOTLOADER_PA2_CAN
     ```

   -   等待终端输出

       ```powershell
       Compiling obj/AM32_G431_BOOTLOADER_PA2_CAN_V16.elf
       Memory region         Used Size  Region Size  %age Used
                  FLASH:       13508 B        16 KB     82.45%
                    RAM:       13776 B        16 KB     84.08%
       Generating obj/AM32_G431_BOOTLOADER_PA2_CAN_V16.hex```
       ```

       AM32 Boot loader 编译成功

### （二）编译 AM32 固件

1. **获取源码**

   - 选择一个工作文件夹，并在工作文件夹中打开 VS Code

   -   打开终端 (`Ctrl + ~`)

   -   输入以下命令拉取 ExpressLRS 源码

       ```powershell
       git clone https://github.com/am32-firmware/AM32.git
       cd AM32
       ```

       注意：如果已经有源码，直接在 VS Code 中 `File -> Open Folder` 打开 `AM32` 文件夹即可。

   -  切换到master分支

2. **安装依赖并配置临时环境变量**

   - 打开终端 (`Ctrl + ~`)

   -   输入以下命令安装依赖

       ```powershell
       cd .\env_setup_scripts\
       .\gcc_windows_env_setup.cmd
       ```

   -   等待终端输出`Script completed successfully.`依赖安装成功

   -   输入以下命令返回根目录并配置临时环境变量

       ```powershell
       cd ..
       $env:PATH = "$PWD\tools\windows\make\bin;$PWD\tools\windows\xpack-arm-none-eabi-gcc-10.3.1-2.3\bin;" + $env:PATH
       ```

3. **修改源码**

   -  修改 `Inc/targets.h`并添加自定义目标定义
     
      在文件末尾添加以下代码
      ```c++
      #ifdef STSPIN_G431_USER_CAN 
      #define FIRMWARE_NAME "STSPIN G431"
      #define FILE_NAME "STSPIN_G431_USER_CAN"
      #define DRONECAN_SUPPORT 1
      #define DRONECAN_NODE_NAME "com.jennifer.esc" 
      #define DEAD_TIME 80 
      
      #define HARDWARE_GROUP_STSPIN_G4 
      #define TARGET_STALL_PROTECTION_INTERVAL 20000
      #define USE_SERIAL_TELEMETRY
      #define USE_ADC_1_2
      #define MILLIVOLT_PER_AMP 25 
      #define NTC_ADC_PIN LL_GPIO_PIN_1 // PB1
      #define NTC_ADC_CHANNEL LL_ADC_CHANNEL_12 // ADC1 IN12
      #define VOLTAGE_ADC_PIN LL_GPIO_PIN_6 // PA6
      #define VOLTAGE_ADC_CHANNEL LL_ADC_CHANNEL_3 // ADC2 IN3
      #define CURRENT_ADC_PIN LL_GPIO_PIN_7 // PA7
      #define CURRENT_ADC_CHANNEL LL_ADC_CHANNEL_4 // ADC2 IN4
      #define WS2812_PIN LL_GPIO_PIN_8 // PB8
      #define USE_LED_STRIP
      #define USE_PULSE_OUT
      #define RPM_PULSE_PORT GPIOB
      #define RPM_PULSE_PIN LL_GPIO_PIN_10
      
      #define USE_HSE 
      #undef HSE_VALUE
      #define HSE_VALUE 8000000
      #define USE_HSE_BYPASS 0
      #endif
      
      #ifdef HARDWARE_GROUP_STSPIN_G4
      #define MCU_G431
      #define USE_TIMER_15_CHANNEL_1
      #define INPUT_PIN LL_GPIO_PIN_2 // PA2 Dshot
      #define INPUT_PIN_PORT GPIOA
      #define IC_TIMER_CHANNEL LL_TIM_CHANNEL_CH1
      #define IC_TIMER_REGISTER TIM15
      #define IC_TIMER_POINTER htim15
      #define INPUT_DMA_CHANNEL LL_DMA_CHANNEL_1
      #define DMA_HANDLE_TYPE_DEF hdma_tim15_ch1
      #define IC_DMA_IRQ_NAME DMA1_Channel1_IRQn
      
      #define PHASE_A_GPIO_LOW LL_GPIO_PIN_8
      #define PHASE_A_GPIO_PORT_LOW GPIOE
      #define AF_A_LOW LL_GPIO_AF_2
      #define PHASE_A_GPIO_HIGH LL_GPIO_PIN_9
      #define PHASE_A_GPIO_PORT_HIGH GPIOE
      #define AF_A_HIGH LL_GPIO_AF_2
      
      #define PHASE_B_GPIO_LOW LL_GPIO_PIN_10
      #define PHASE_B_GPIO_PORT_LOW GPIOE
      #define AF_B_LOW LL_GPIO_AF_2
      #define PHASE_B_GPIO_HIGH LL_GPIO_PIN_11
      #define PHASE_B_GPIO_PORT_HIGH GPIOE
      #define AF_B_HIGH LL_GPIO_AF_2
      
      #define PHASE_C_GPIO_LOW LL_GPIO_PIN_12
      #define PHASE_C_GPIO_PORT_LOW GPIOE
      #define AF_C_LOW LL_GPIO_AF_2
      #define PHASE_C_GPIO_HIGH LL_GPIO_PIN_13
      #define PHASE_C_GPIO_PORT_HIGH GPIOE
      #define AF_C_HIGH LL_GPIO_AF_2
      
      #define PHASE_A_COMP LL_COMP_INPUT_MINUS_IO1 // PA5 (COMP2 INM)
      #define PHASE_A_INPUT_PLUS LL_COMP_INPUT_PLUS_IO2 // PA3 (VNP)
      #define PHASE_A_EXTI_LINE LL_EXTI_LINE_22
      #define PHASE_A_COMP_NUMBER COMP2
      
      #define PHASE_B_COMP LL_COMP_INPUT_MINUS_IO1 // PA4 (COMP1 INM)
      #define PHASE_B_INPUT_PLUS LL_COMP_INPUT_PLUS_IO1 // PA1 (VNP)
      #define PHASE_B_EXTI_LINE LL_EXTI_LINE_21
      #define PHASE_B_COMP_NUMBER COMP1
      
      #define PHASE_C_COMP LL_COMP_INPUT_MINUS_IO2 // PA0 (COMP1 INM)
      #define PHASE_C_INPUT_PLUS LL_COMP_INPUT_PLUS_IO1 // PA1 (VNP)
      #define PHASE_C_EXTI_LINE LL_EXTI_LINE_21
      #define PHASE_C_COMP_NUMBER COMP1
      #endif
      ```
      
   - 修改 `Mcu/g431/Src/peripherals.c`以适配 PWM 引脚复用功能和 PA0 初始化

     1. 在 `MX_COMP1_Init` 函数中，添加 PA0 的 GPIO 初始化
        ```c++
           GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
           GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
           GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
           LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        // --- 添加以下代码 ---
        #ifdef HARDWARE_GROUP_STSPIN_G4
           GPIO_InitStruct.Pin = LL_GPIO_PIN_0; 
           GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
           GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
           LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        #endif
        // -------------------
        
           COMP_InitStruct.InputPlus = LL_COMP_INPUT_PLUS_IO1;
        ```

     2. 在 `MX_TIM1_Init` 函数末尾的 GPIO 配置部分，修改 Phase A/B/High 的 AF 映射
        
        *找到 `PHASE_A_GPIO_HIGH` 的配置块，修改 `GPIO_InitStruct.Alternate`*
        
        ```c++
        // 在 GPIO_InitStruct.Pin = PHASE_A_GPIO_HIGH 之后
        #ifdef HARDWARE_GROUP_STSPIN_G4
            GPIO_InitStruct.Alternate = AF_A_HIGH; // AF_2
        #else
            GPIO_InitStruct.Alternate = LL_GPIO_AF_6; // 默认
        #endif
        ```
        *对 `PHASE_B_GPIO_HIGH` 和 `PHASE_C_GPIO_HIGH` 做同样的修改*

   - 修改 `Mcu/g431/Src/ADC.c`以修复 NTC 采样时间配置 Bug

     在 `ADC_Init` 函数的 `#ifdef USE_ADC_1_2` 块中
     ```c++
     LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, NTC_ADC_CHANNEL);
     LL_ADC_SetChannelSamplingTime(ADC1, NTC_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_47CYCLES_5); 
     LL_ADC_SetChannelSingleDiff(ADC1, NTC_ADC_CHANNEL, LL_ADC_SINGLE_ENDED); 
     ```
     
   - 修复 Windows 下 `python3` 命令找不到的问题
     将 `Makefile` 这里的 `python3` 替换为 `python`
     
     ```makefile
     # 在 $(QUIET)$(xOBJCOPY) -O binary $$(<) $$@ 之后
     $(QUIET)python Src/DroneCAN/set_app_signature.py $$@ $$(<)
     ```

4. **编译固件**

   - 打开终端 (`Ctrl + ~`)

   - 输入以下命令开始编译自定义目标

     ```powershell
     make STSPIN_G431_USER_CAN
     ```

   - 等待终端输出

     ```powershell
     Compiling AM32_STSPIN_G431_USER_CAN_2.20.elf
     Memory region         Used Size  Region Size  %age Used
                  RAM:        8200 B        32 KB     25.02%
               FLASH1:          1 KB         1 KB    100.00%
            FILE_NAME:          32 B         32 B    100.00%
               EEPROM:          0 GB         2 KB      0.00%
                FLASH:       55185 B     111584 B     49.46%
     echo building BIN obj/AM32_STSPIN_G431_USER_CAN_2.20.bin
     building BIN obj/AM32_STSPIN_G431_USER_CAN_2.20.bin
     Generating AM32_STSPIN_G431_USER_CAN_2.20.bin
     Applied APP_DESCRIPTOR 562c578e6e33aafb for obj/AM32_STSPIN_G431_USER_CAN_2.20.bin
     ```

     AM32 固件编译成功

## 三、烧录固件

   1. **烧录 AM32 Boot loader**

      - 使用 ST-Link 或者 DAP-LinK 或者 J-Link 和 4Pin 1.5mm 间距 PCB 测试夹连接电调板与电脑，确保电调已经上电

      - 打开终端 (`Ctrl + ~`)

      - 输入以下命令
      
        ```powershell
        .\tools\windows\openocd\bin\openocd -s tools\windows\openocd\share\openocd\scripts -f obj\openocd.cfg -c "program obj\AM32_G431_BOOTLOADER_PA2_CAN_V16.hex verify reset exit"
        ```
         等待命令完成即可
      
      - 也可以使用`STM32CubeProgrammer`烧录
      

   2. **烧录 AM32 固件**

      - 使用 USB-TTL 模块或者 Arduino 连接电调板 PA2 与电脑，确保电调已经上电

      - 从 [AM32 官网](https://am32.ca/) 下载 Configurator

      - 在 Configurator 中连接电调板，选择 `Flash Firmware`，然后加载 `obj/AM32_STSPIN_G431_USER_CAN_2.20.hex` 文件等待烧录完成即可

## 四、调试

1. 使用 USB-TTL 模块或者 Arduino 连接电调板 PA2 与电脑，确保电调已经上电
2. 在 Configurator 中连接电调板，即可修改配置参数