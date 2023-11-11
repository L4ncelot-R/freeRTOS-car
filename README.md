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

1. [motor_init](frtos/motor/motor_init.h): Initialises the pins and struct stated in [motor_config](frtos/config/motor_config.h) and [car_config](frtos/config/car_config.h), and the free rtos tasks related. Call `motor_init` followed by `motor_tasks_init` to start.
2. [motor_speed](frtos/motor/motor_speed.h): Monitors the speed of the motors by interrupt. Each **falling edge** of the wheel encoder will trigger the interrupt and the time elapsed since last trigger will be recorded and thus calculate the speed and distance travelled since boot. Includes functions to adjust the duty cycle of the motor.
3. [motor_pid](frtos/motor/motor_pid.h): PID function to match the **right** motor duty cycle to the **left** motor such that the car can move straight, with 50ms interval.
4. [motor_direction](frtos/motor/motor_direction.h): Sets the direction of the motor to control the car using bit masks. Includes functions that work with the magnetometer to turn the car to/by specific yaw.

## Ultrasonic Sensor

## Line Sensor

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

## WiFi (Web Server) Module
