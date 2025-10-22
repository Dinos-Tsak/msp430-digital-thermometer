# msp430-digital-thermometer

# MSP-EXP430FR5994 LM75A Temperature Sensor Project

This project demonstrates how to read temperature data from an **LM75A digital temperature sensor** via **I²C** and display the readings on a computer screen via **UART**, using the **MSP-EXP430FR5994 LaunchPad**.  
The code is written in C and developed using **Code Composer Studio (CCS)**.

---

## 🔧 Description

The program communicates with the **LM75A** sensor using the **I²C protocol** to read temperature measurements. These readings are then transmitted over **UART** to a PC for display.  

This project is useful for understanding:

- I²C communication with sensors on MSP430  
- GPIO and peripheral setup on MSP430FR5994  
- UART transmission for monitoring and debugging  

---

## 🧰 Hardware Requirements

| Component | Description |
|-----------|-------------|
| MSP-EXP430FR5994 LaunchPad | TI development board with MSP430FR5994 MCU |
| LM75A | Digital temperature sensor with I²C interface |
| Pull-up resistors | 4.7 kΩ for SDA and SCL lines |
| USB Cable | For power and UART communication |
| Jumper Wires | For wiring connections |

---

## 🔌 Wiring Connections

| LaunchPad Pin | LM75A Pin | Description |
|---------------|-----------|-------------|
| 3.3 V | VCC | Power supply |
| GND | GND | Ground |
| P3.0 | SDA | I²C data line (connect via 4.7 kΩ pull-up to 3.3 V) |
| P3.1 | SCL | I²C clock line (connect via 4.7 kΩ pull-up to 3.3 V) |
| P2.0 | UART TX | Connect to PC for serial output |

> Adjust pins according to your actual wiring in `main.c`

---

## 💻 Software Requirements

- **Code Composer Studio (CCS)** → [https://www.ti.com/tool/CCSTUDIO](https://www.ti.com/tool/CCSTUDIO)  
- **MSP430Ware / DriverLib** (included in CCS)  
- **USB Drivers** for LaunchPad  
- **Serial Terminal Program** (e.g., PuTTY, Tera Term, minicom)

---

## ⚙️ Setup and Usage Instructions

1. **Clone the repository:**
   ```bash
   git clone https://github.com/username/msp430fr5994-lm75a-i2c.git
