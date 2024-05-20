<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
</head>
<body>
    <h1>STM32 Project: Interfacing ADXL345 using SPI and MPU6050 using I2C</h1>
    <h2>Overview</h2>
    <p>This project demonstrates how to interface an ADXL345 accelerometer using the SPI protocol and an MPU6050 accelerometer and gyroscope using the I2C protocol with an STM32 microcontroller. It includes initializing the communication protocols, reading sensor data, and processing the data for further use.</p>
    <h2>Features</h2>
    <ul>
        <li>Initialize and configure SPI for ADXL345 communication.</li>
        <li>Initialize and configure I2C for MPU6050 communication.</li>
        <li>Read acceleration data from the ADXL345 sensor.</li>
        <li>Read acceleration and gyroscope data from the MPU6050 sensor.</li>
        <li>Process and display sensor data.</li>
    </ul>
    <h2>Hardware Requirements</h2>
    <ul>
        <li>STM32 microcontroller (e.g., STM32F103C8T6).</li>
        <li>ADXL345 accelerometer sensor.</li>
        <li>MPU6050 accelerometer and gyroscope sensor.</li>
        <li>Connecting wires and breadboard.</li>
    </ul>
    <h2>Software Requirements</h2>
    <ul>
        <li>STM32CubeMX for generating initialization code.</li>
        <li>STM32CubeIDE or Keil uVision for coding and debugging.</li>
        <li>HAL (Hardware Abstraction Layer) libraries for STM32.</li>
    </ul>
    <h2>Setup and Configuration</h2>
    <h3>ADXL345 (SPI)</h3>
    <p>Configure the SPI settings in STM32CubeMX:</p>
    <ul>
        <li>SPI Mode: Master</li>
        <li>Baud Rate: Suitable for your application (e.g., 1 MHz)</li>
        <li>Data Size: 8 bits</li>
        <li>Clock Polarity and Phase: As required by ADXL345</li>
        <li>Enable NSS (Chip Select) pin</li>
    </ul>
    <p>Connections:</p>
    <ul>
        <li>ADXL345 VCC to STM32 3.3V</li>
        <li>ADXL345 GND to STM32 GND</li>
        <li>ADXL345 CS to STM32 SPI NSS</li>
        <li>ADXL345 SDO/ALT ADDRESS to STM32 SPI MISO</li>
        <li>ADXL345 SDA/SDI/SDIO to STM32 SPI MOSI</li>
        <li>ADXL345 SCL/SCLK to STM32 SPI SCK</li>
    </ul>
    <h3>MPU6050 (I2C)</h3>
    <p>Configure the I2C settings in STM32CubeMX:</p>
    <ul>
        <li>I2C Mode: I2C</li>
        <li>Timing: As required by MPU6050 (e.g., 100 kHz for standard mode)</li>
    </ul>
    <p>Connections:</p>
    <ul>
        <li>MPU6050 VCC to STM32 3.3V</li>
        <li>MPU6050 GND to STM32 GND</li>
        <li>MPU6050 SDA to STM32 I2C SDA</li>
        <li>MPU6050 SCL to STM32 I2C SCL</li>
    </ul>
    <h2>Code Implementation</h2>
    <p>Here is an outline of the main steps in the code:</p>
    <ol>
        <li>Initialize the HAL library and configure the system clock.</li>
        <li>Initialize SPI and I2C peripherals.</li>
        <li>Configure and initialize ADXL345 and MPU6050 sensors.</li>
        <li>Read data from the sensors.</li>
        <li>Process and use the sensor data as required.</li>
// Initialize SPI for ADXL345
      use the include adxl driver for intizialization and reading of data,and check the function of the driver.
// Initialize I2C for MPU6050
    <h2>Contributing</h2>
    <p>Contributions are welcome! To contribute, please follow these steps:</p>
    <ol>
        <li>Fork the repository and create a new branch.</li>
        <li>Make your changes and test them.</li>
        <li>Submit a pull request with a clear explanation of your changes.</li>
    </ol>
</body>
</html>
