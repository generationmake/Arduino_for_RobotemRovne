# Arduino_for_RobotemRovne
Arduino Code for Robotem Rovne Contest.

## boards used

* [Arduino MKR WiFi 1010](https://store.arduino.cc/products/arduino-mkr-wifi-1010)
* [HighPowerMotorFeatherWing](https://github.com/generationmake/HighPowerMotorFeatherWing) (with MKR adapter)
* [ArduHMIShield](https://github.com/generationmake/ArduHMIShield) (with MKR header and DOGM128-6 display)
* [NavShieldBNO](https://github.com/generationmake/NavShieldBNO) (BNO055 variant)


## pin usage

| **Pin** | **Pin Name** | **board**                 | **Signal**    | **Description**                                 |
|:-------:|:------------:|:-------------------------:|:-------------:|:-----------------------------------------------:|
| 1       | AREF         |                           |               |                                                 |
| 2       | DAC0/A0      | HighPowerMotorFeatherWing | BAT_VOLTAGE   | analog value of battery voltage                 |
| 3       | A1           | HighPowerMotorFeatherWing | MOT1_EN       | enable for motor 1                              |
| 4       | A2           | HighPowerMotorFeatherWing | MOT2_EN       | enable for motor 2 (make free for ArduHMI Keypad pin) |
| 5       | A3           |                           |               |                                                 |
| 6       | A4           | HighPowerMotorFeatherWing | EMERGENCYSTOP | input for state of emergency stop               |
| 7       | A5           |                           |               |                                                 |
| 8       | A6           | ArduHMIShield             | BACKLIGHTPIN  | backlight for display                           |
| 9       | 0            | ArduHMIShield             | DIS_A0        | mode select for display                         |
| 10      | 1            | ArduHMIShield             | DIS_RESET     | reset for display                               |
| 11      | 2            | HighPowerMotorFeatherWing | MOT1_1        | pwm for motor 1                                 |
| 12      | 3            | HighPowerMotorFeatherWing | MOT1_2        | pwm for motor 1                                 |
| 13      | 4            | HighPowerMotorFeatherWing | MOT2_1        | pwm for motor 2                                 |
| 14      | 5            | HighPowerMotorFeatherWing | MOT2_2        | pwm for motor 2                                 |
| 15      | 6            | ArduHMIShield             | DIS_CS        | chip select for display                         |
| 16      | 7            |                           |               |                                                 |
| 17      | 8/MOSI       | several                   | SPI_MOSI      | SPI for display                                 |
| 18      | 9/SCK        | several                   | SPI_SCK       | SPI for display                                 |
| 19      | 10/MISO      | several                   | SPI_MISO      | SPI for display                                 |
| 20      | 11/SDA       | several                   | I2C_SDA       | I2C for BNO                                     |
| 21      | 12/SCL       | several                   | I2C_SCL       | I2C for BNO                                     |
| 22      | 13/RX        | reserved                  | SERIAL_RX     | reserved for GPS                                |
| 23      | 14/TX        | reserved                  | SERIAL_TX     | reserved for GPS                                |
| 24      | RESET        | several                   | RESET         | Reset for BNO                                   |
| 25      | GND          | all                       | GND           | common ground for all boards                    |
| 26      | VCC          | all                       | 3V3-rail      | supply voltage for all boards                   |
| 27      | VIN          | HighPowerMotorFeatherWing | 5V-rail       | output of voltage generator, supply for Arduino |
| 28      | 5V0          |                           |               |                                                 |
