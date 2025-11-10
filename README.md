# Embedded-RTOS
Real Time Operating System Project for ECE-4380

# Features
* Custom RTOS
    * Custom Real-Time Operating System built upon the internal timer of the STM32, and without external packages
* Dual Power Monitoring
    * Samping INA219s via I2C
* Bluetooth Streaming
    * Send formatted power readings to the STM32 over UART
* Website Hosting
    * Laptop-hosted webpage for information display and light control
* Light Control
    * Turning on & off lights based on phototransistor readings or webpage inputs
* Multi-room Extension
    * Two HC-05s link to seperate lamps. Local lamp readings or webpage inputs will turn on specified lamps

# System Requirements
In order to run this RTOS, you will need:
* Hardware
    * MCUs
       * STM32 Nucleo - STM32F446RET6 (Primary Controller)
       * STM32 Blue Pill (Remote Node)
   * Sensors
       * 2x INA219 I2C Current/Voltage/Power Sensors
       * 2x USB-A 3.0 Power Monitors
   * Wireless
       * 2x HC-05 Bluetooth SPP UART Modules
   * Actuators
       * 2x CQC SRD-5VDC-SL-C Logic-Level Relays
   * Power
       * 2x USB Wall Adapters (5V / 2A)
   * Laptop with Bluetooth Capability
* Software
   * STM32CubeIDE 
      * stm32f1xx_hal C package (for STM32 functionality)
      * EGit (for version control)
 
