# Avionics
  The Avionivs package is the code and hardware that is responsible for gathering and logging data from various sensors for the purpose of monitoring the performance/behavior of the aircraft. It will also execute any required tasks depending on the mission (e.g. driving servos to open/close doors for sensor package deployment). The Arduino IDE along with several libraries that are listed below are used to develop code. All of the libraries can be directly downloaded from the "Manage Libraries..." option under "tools" in the IDE.

- [Servo](https://www.arduino.cc/reference/en/libraries/servo/)
- [Wire](https://www.arduino.cc/en/reference/wire)
- [SparkFun_BNO080_Arduino_Library](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library)


## Microprocessor
ATSAMD51 [MicroMod SAMD51 Processor](https://www.sparkfun.com/products/16791) SAM D ARM® Cortex®-M4 MCU 32-Bit Embedded Evaluation Board
[![SparkFun MicroMod SAMD51 Processor (DEV-16791)](https://github.com/UF-Design-Build-Fly/Avionics/blob/main/Figures/SAMD51_MicroMod.PNG)](https://www.sparkfun.com/products/16791)

## Carrier Board
SparkFun [MicroMod ATP Carrier Board](https://www.sparkfun.com/products/16885)

[![SparkFun MicroMod ATP Carrier Board (DEV-16885)](https://github.com/UF-Design-Build-Fly/Avionics/blob/main/Figures/Carrier_Board.PNG)](https://www.sparkfun.com/products/16885)

## The package will include readings for:
- Temperature: [10k Thermistors](https://www.digikey.com/en/products/detail/MF52A2103J3470/317-1258-ND/1191033?itemSeq=352912458) and/ or [digital temperature sensors](https://www.adafruit.com/product/1782?gclid=Cj0KCQiAx9mABhD0ARIsAEfpavTrRpLu3mh6pQtUoUhwfdHJb2bvFCE8ZoAFT9-lS7LuWngm052WwCQaAsf0EALw_wcB)
- [Analog current sensor](https://www.sparkfun.com/products/16408)
- Voltage (Analog pin with voltage divider)
- Angle of attack: Rotary encoder
- Orientation: [IMU](https://www.sparkfun.com/products/14686)
- [GPS](https://www.sparkfun.com/products/17285) with an [antenna](https://www.sparkfun.com/products/177)
- [Airspeed sensor](https://www.racedayquads.com/products/matek-analog-airspeed-sensor-aspd-4525?currency=USD&gclid=Cj0KCQiAyp7yBRCwARIsABfQsnS2yW_IQ5EsHOuLm1RxqYPqBrf-bKbCy7pdZkK-Nq9PGUy9E7IiIWsaAsmwEALw_wcB&variant=30112599146609)
- Inputs from the receiver (pwm)
- [Data logging](https://www.digikey.com/en/products/detail/BOB-00544/1568-1345-ND/5824094?itemSeq=352913504) and an [8 GB micro SD card]()
- Hall Sensor [Latching](https://www.sparkfun.com/products/9312)/[Non-Latching](https://www.sparkfun.com/products/14709)
