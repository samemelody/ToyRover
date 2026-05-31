# ToyRover

STM32F103C8T6-based embedded robotics rover running FreeRTOS (CMSIS-RTOS V2), communicating with a mobile app via BLE.

## Hardware

| Component | Detail |
|-----------|--------|
| MCU | STM32F103C8T6 (Cortex-M3, 64KB Flash, 20KB RAM) |
| Bluetooth | MLT-BT05 (CC2541 BLE 4.0), USART1, 9600-8N1 |
| USB | Full-speed device (internal pull-up) |
| Debug | SWD (ST-Link V2.1) + SWO/ITM printf output |

## Build

```bash
# Requires: arm-none-eabi-gcc, CMake, Ninja
cmake --preset Debug
cmake --build build/Debug
```

## Flash & Debug

Uses Cortex-Debug + OpenOCD. `printf` output via SWO console. Press F5 to build, flash, and debug.

## Directory Layout

```
ToyRover/
├── Core/              # Application code (main.c, HAL MSP, ISRs, protocol layer)
├── Drivers/           # CMSIS + STM32F1 HAL
├── Middlewares/       # FreeRTOS kernel + CMSIS-RTOS V2
├── cmake/             # Toolchain + CubeMX CMake fragments
├── CMakeLists.txt
├── STM32F103XX_FLASH.ld
└── ToyRover.ioc       # CubeMX project file
```

## Pin Map

| Pin | Function |
|-----|----------|
| PA9 | USART1_TX → BLE RXD |
| PA10 | USART1_RX → BLE TXD |
| PA11 | USB_DM |
| PA12 | USB_DP |
| PA13 | SWDIO |
| PA14 | SWCLK |
| PB3 | SWO (printf output) |
