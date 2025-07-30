# A project submitted for Bosch intern final assignment about implementing basic CAN bus protocol

## Include of:
### Project1: Basic CAN Data Frame transmit with CRC field using J1850 algorithm
### Project2: DCM (Diagnostic Communication Manager) and UDS (Unified Diagnostic Services) with 3 services: 22, 27, 2E

## Usage:
- BEA_HUST_2025 folder is working properly for project 2, which is the UDS along with ISO-TP Multiple Frame.
- If you test the UDS, nothing need to be changed. Just clean build the whole project inside folder and run.
- Because 2 projects are separated, i put the main.c file for project 1 outside (project1.c and project1.h), just replace them with the main.c and main.h inside the BEA_HUST_2025/Core/Src/ and BEA_HUST_2025/Core/Inc. Then delete all the dcm files along with iso_tp (both .c and .h)
- Remembember to clean build again if switch to project1

## Preresiquites:
- Our project implement on a Open405R-C kit board (STM32), a pair of CAN bus modules plugged into both CAN1 and CAN2 port of the kit board, connected together with a pair of twisted wire, and debug on computer terminal via UART3 port using a UART module and tio CLI tool (but you can use any other serial monitor tool like Hercules, Terra Term). Programming and config clock, timing with STM32CubeIDE.

