# Smart Motor Controller

A learning project for DC motor speed control on STM32F103C6 using C, STM32 HAL and FreeRTOS.

Features: PWM and PID control with encoder feedback, forward/reverse rotation, CAN or UART commands and telemetry, current monitoring, emergency stop, communication timeout, stall detection and IWDG watchdog. Two FreeRTOS tasks communicate with RX interrupts through direct task notifications; application logic is in `Core/Src/main.c`.

## Build and run

1. Import `SmartMotor` into STM32CubeIDE.
2. CAN is enabled by default. For UART, uncomment `#define COMM_MODE_UART` in `main.c`.
3. Run Clean → Build and flash the board using ST-Link.

Hardware requires a motor driver, encoder and current sensor; CAN also requires a transceiver. Check wiring, PID gains, current conversion and the configured 96 encoder counts per revolution before running.

## Commands

CAN: 500 kbit/s, standard data frames, command ID `0x010`, telemetry ID `0x020`. UART: 115200 baud, 8N1. Send one complete ASCII command per CAN frame or UART write; UART commands may end with `\n`.

| Command | Action |
| --- | --- |
| `F200` / `R200` | Forward / reverse at a target of 200 RPM; range 1–999 |
| `STOP`, `F0`, `R0` | Stop |
| `RESET` | Clear faults after their cause is removed; remain stopped |
| `CRASH` | Stop and hang the command task to test the watchdog |

While moving, repeat the speed command every 200 ms; a timeout over 500 ms stops the motor. To reverse, send `STOP`, wait at least 100 ms and until speed is at most 10 RPM in magnitude, then send the opposite command.

Only one command can wait for processing; an additional command while it is pending triggers a communication fault and stops the motor.

## Status

CAN and UART builds passed. Hardware operation, sensor calibration and PID tuning still need verification.
