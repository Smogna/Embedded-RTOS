# Embedded-RTOS
Real Time Operating System Project for ECE-4380

# Features
* Custom RTOS
    * Custom Real-Time Operating System built upon the internal timer of the STM32, and without external packages
* Dual Power Monitoring
    * Samping INA219s via I2C
* Bluetooth Streaming
    * Send formatted power readings to the website via HC-05s and UART
* Website Hosting
    * Laptop-hosted webpage for information display and light control
* Light Control
    * Turning on & off lights based on user inputs

# System Requirements
In order to run this RTOS, you will need:
* Hardware
    * MCUs
       * STM32 Nucleo - STM32F446RET6 (Primary Controller)
       * STM32 Blue Pill (Remote Node)
   * Sensors
       * 1x INA219 I2C Current/Voltage/Power Sensors
   * Wireless
       * 1x HC-05 Bluetooth SPP UART Modules
   * Actuators
       * 1x CQC SRD-5VDC-SL-C Logic-Level Relays
   * Laptop with Bluetooth Capability
* Software
   * STM32CubeIDE 
      * stm32f1xx_hal C package (for STM32 functionality)
    
    # Run and Build Instructions
  In order to have a functioning project:
  * Step 1
     * Download the RTOS.c and index.html webpage
  * Step 2
     * Import the files into either the STM32CubeIDE or Arduino IDE
    * Step 3
       * Flash the board with the code
    * Step 4
       * Launch the webpage and connect to the HC05
      * EGit (for version control)
 
