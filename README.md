# SensorSystems-Polimi
Varied small projects developed for an STM-32 microcontroller exploring the use of different communication protocols to interact with sensors and output peripherals.

## Notable projects
### 1. L11
Detect which button was pressed on an SPI based keyboard matrix and send corresponding character to be displayed on an LED matrix using IR communication.
### 2. exam
Read temperature from a smart sensor using I2C and compare it to a threshold value received through a remote terminal in real time (using UART) to toggle an LED.
### 3. Kb_Sound
Play a different sound through a speaker every time a specific key is pressed on an SPI based keyboard matrix.
### 4. exam_simulation2
Read temperature values from both a smart sensor (using I2C) and the internal microcontroller temperature sensor (with an ADC). Print both values on an LCD display and send highest value to a remote terminal using UART.
### 5. led_intensity
Control intensity of an LED based on values read from a potentiometer.
