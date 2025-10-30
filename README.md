Spiderbot Project

This Spiderbot project uses the STM32F103 microcontroller to control a spider robot (spiderbot), which performs the following functions:
  Battery Monitoring with ADC: Uses ADC to measure the system's battery voltage.
  Display Battery Status on LCD: The battery level is shown on an I2C LCD screen.
  Control via HC-05 Bluetooth Module: The robot can be controlled via Bluetooth using the HC-05 module.
  PCA9685 Servo Control: The PCA9685 is used to control the servo motors, enabling the robot's movement.

Key Components of the Project:

  STM32F103: The central microcontroller of the system.
  ADC: Measures the battery voltage and converts it to a numerical value.
  I2C LCD: Displays the system's battery status.
  HC-05 Bluetooth Module: Used to receive control signals from an external device such as a smartphone.
  PCA9685: Controls the servo motors for robot movement.
  Servo Motors RC Servo 9G 0-180 : Used to move the robot's legs.

├── adc.c                  # Measures battery voltage using ADC
├── gpio.c                 # Manages STM32F103 GPIO pins
├── i2c.c                  # I2C communication with LCD
├── lcd_i2c.c              # Controls the LCD via I2C
├── main.c                 # Main program of the robot
├── pca9685.c              # Controls the PCA9685
├── stm32f1xx_hal_msp.c    # STM32F103 hardware configuration
├── stm32f1xx_it.c         # Interrupt handling
├── syscalls.c             # System support functions
├── sysmem.c               # Memory management
├── system_stm32f1xx.c     # STM32F103 system configuration
├── tim.c                  # Timer management
├── usart.c                # UART communication for HC-05


