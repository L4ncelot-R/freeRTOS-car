# freeRTOS-car

This project is a car that uses the [Raspberry Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico/)
and [FreeRTOS](https://www.freertos.org/) to move around a maze.

# Configurations

All Configurations are in the folder `frtos/config`, the file `frtos/config/car_config.h` contains the main
configurations for the car.

# Build

> Configure your Pico SDK and FreeRTOS Kernel Path in your local environment

```bash
mkdir build
cd build
cmake ..
make
```

# Flash

```bash
cd build
cp freeRTOS-car.uf2 /media/$USER/RPI-RP2
```

# Components
## Motors
Motor module consist of 4 components:

1. [motor_init](frtos/motor/motor_init.h): Initialises the pins and struct stated in [motor_config](frtos/config/motor_config.h) and [car_config](frtos/config/car_config.h), and the free rtos tasks related. Call `motor_init` followed by `motor_tasks_init` to initilise the parameters and define the tasks.
2. [motor_speed](frtos/motor/motor_speed.h): Monitors the speed of the motors by interrupt. Each **falling edge** of the wheel encoder will trigger the interrupt and the time elapsed since last trigger will be recorded and thus calculate the speed and distance travelled since boot. Typically, each interrupt will record one slot length of the distance travelled, calculated and defined in [motor_config](frtos/config/motor_config.h). Includes functions to adjust the duty cycle of the motor for speed control.
3. [motor_pid](frtos/motor/motor_pid.h): PID function to match the one motor duty cycle to the other motor such that the car can move straight, with 50ms interval, by comparing the difference in distance travelled by each wheel.
4. [motor_direction](frtos/motor/motor_direction.h): Sets and changes the direction of the motor to control the car using bit masks. Includes functions that work with the magnetometer to turn the car to/by specific yaw.

## Ultrasonic Sensor
1. Ultrasonic sensor package is in `frtos/ultrasonic_sensor`, and its configuration is in `frtos/config/ultrasonic_sensor_config.h`. It contains the drivers and FreeRTOS tasks to output and read the ultrasonic sensor data.

2. [ultrasonic_init](frtos/ultrasonic_sensor/ultrasonic_init.h): Initialises the pins and struct stated in [ultrasonic_sensor_config](frtos/config/ultrasonic_sensor_config.h) and [car_config](frtos/config/car_config.h), and the free rtos tasks related. Call `ultrasonic_init` followed by `ultrasonic_tasks_init` to start.

3. The function check_obstacle() updates one of the parameter in the car's obstruction struct (s_obs_struct) in [car_config](frtos/config/car_config.h) to tell the car's whether it detects an obstruction . 

## Line Sensor

The Line sensor package is in `frtos/line_sensor`, and its configuration is in `frtos/config/line_sensor_config.h`. It
contains the drivers and FreeRTOS tasks to read the line sensor data and update the car's obstruction struct.

## Magnetometer

The magnetometer used is the [LSM303DLHC](https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf) from STMicroelectronics.

Magnetometer package is in `frtos/magnetometer`, and its configuration is in `frtos/config/magnetometer_config.h`. It 
contains the drivers and FreeRTOS tasks to read the magnetometer data.

All the magnetometer data (roll, pitch, yaw) calculated is stored in the `direction_t` struct, which is defined in
`frtos/config/magnetometer_config.h`.

### Filtering

The magnetometer initially used a complementary filter (with the accelerometer) to calculate the Yaw, and the temperature,
sensor was used to perform an offset correction. But raw data was accurate enough to use it directly. 

An initial calibration method for the magnetometer was implemented, with the use of 100 initial samples, and then
a bias calculation. This method was not used in the final version, because the raw data was accurate enough.

The final version of the magnetometer uses a **moving average filter** to smooth the data and a bias of 0 degrees.

## Barcode (Line) Sensor
The barcode sensor uses edge interrupts to trigger the barcode sensor when the sensor hits a black line, which then measures the width of the black line to white, and stores it in an array. The Code39 barcode library is used as a reference for the barcode decoding function.

## WiFi (Web Server) Module

The WiFi (Web Server) Module within this project integrates Server-Side Includes (SSI) and Common Gateway Interface (CGI) handlers from the lwIP library, facilitating a web interface to display information and receive controls from a webpage.

### Components Overview

1. **[frontend.h](frtos/frontend/frontend.h):** This component initializes and manages the web server frontend. It establishes the initial connection to the WiFi network using environment variables such as WIFI_SSID and WIFI_PASSWORD.

2. **[html_files](frtos/frontend/html_files):** This directory contains HTML files used for the web server display. These files are linked into lwIP to enable server-side processing and rendering.

3. **[ssi.h](frtos/frontend/ssi.h):** Handles Server-Side Includes (SSI), which dynamically inserts content into web pages, allowing the display of real-time or changing information.

4. **[cgi.h](frtos/frontend/cgi.h):** Manages Common Gateway Interface (CGI) handlers, enabling the reception of controls and user interactions from the web interface.

5. **makefsdata Utility:** This tool generates `htmldata.c` by converting HTML files into hexadecimal representation to link them into lwIP. It involves converting filenames into hex, defining HTTP headers for file extensions, and converting HTML content into a hex array for efficient storage and retrieval.

    - **Automatic Generation of htmldata.c:**
        - `makefsdata.c` creates `makefsdata.exe` automatically upon build, as specified in `CMakeLists.txt`.
        - `makefsdata.exe` in turn generates `htmldata.c`, which links the HTML pages to lwIP for server-side processing, enabling seamless integration of web content.

### Functionality and Implementation Details

The WiFi (Web Server) Module offers the following functionalities:

- **Dynamic Content Display:** SSI allows the server to include dynamic content in web pages, facilitating real-time updates or information display.

- **User Interaction Handling:** CGI enables the server to process user interactions received from the web interface, enabling control and interaction with the Pico-based car.

- **WiFi Connectivity:** `frontend.h` establishes a connection to the designated WiFi network using the environment variables WIFI_SSID and WIFI_PASSWORD. This connection is essential for enabling communication between the Pico device and the network.


The integration of SSI, CGI, and the makefsdata tool empowers the WiFi (Web Server) Module to provide a user-friendly interface for monitoring and controlling the Pico-based car within the defined network environment.

