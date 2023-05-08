# sc880

Firmware for an STM32F4-based dev board attached inside a Roland SC-880. It is meant to add USB functionality, audio recording and MIDI in/out.

## Details
- Development board used: Olimex STM32-H405
- Programming interface: J-Link
- Using [unicore-mx](https://github.com/insane-adding-machines/unicore-mx) as firmware library.

## Physical connections:
- PB12 - I2S_WS - left/right select
- PB13 - I2S_CK - clock
- PB15 - I2S_SD - data
- PA9 - USART1_TX

## Signal source
- It is tapped off from the input to the main SC-880 DAC, [AK4324VF 24-pin VSOP](https://media.digikey.com/pdf/Data%20Sheets/AKM%20Semiconductor%20Inc.%20PDFs/AK4324.pdf).
- There is also a secondary DAC
- The DAC seems to be set to 20 bit mode
- WS is 32kHz
- CK is 2.048MHz
- data seems to be 20 bit

## Building
Project is meant to go into the `unicore-mx-examples/examples/stm32/f4/stm32f4-discovery` directory.
