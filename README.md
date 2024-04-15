# Sensor Data Display

This project reads data from temperature and humidity sensors (DHT11), an MQ-135 sensor for air quality, and a Sharp PM2.5 sensor for particulate matter measurements.

## Overview

The code is designed to work with an Arduino board and the following sensors:
- DHT11 for temperature and humidity
- MQ-135 for CO, alcohol, CO2, Toluen, NH4, and Aceton measurements
- Sharp PM2.5 sensor for particulate matter (PM2.5) concentration

## Setup

### Hardware Requirements
- Arduino board (e.g., Nano)
- DHT11 temperature and humidity sensor
- MQ-135 air quality sensor
- Sharp PM2.5 sensor
- Wiring components as per the code comments

### Software Requirements
- Arduino IDE for code compilation and uploading

## Installation

1. Connect the sensors according to the provided pin definitions in the code.
2. Install the necessary libraries:
   - DFRobot_DHT11 for DHT11 sensor
   - MQUnifiedsensor for MQ-135 sensor
   - Adafruit SH1106 for OLED display

## Usage

1. Open the Arduino IDE.
2. Copy and paste the code into a new sketch.
3. Compile and upload the sketch to your Arduino board.
4. Open the serial monitor to view sensor readings.

## Additional Information

- Ensure proper calibration and setup for accurate sensor readings.
- Refer to the code comments for sensor-specific configurations.
- For any issues or questions, refer to the Arduino documentation or seek community support.

## Author

Ishan Sehgal

## License

This project is licensed under the [License Name] - see the [LICENSE](LICENSE) file for details.

![Sensor Setup](images/1.png)
![Sensor Setup](images/2.png)

