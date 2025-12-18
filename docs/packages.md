# List of Packages and their functionalities

## [nucleo_interface](../src/nucleo_interface) 
### Features
- Interfaces with the nucleo hardware
- Handles initializing the power mode
- Translates IMU data to publish to /imu
- Takes /cmd_vel commands and sends serial commands to move the steering and wheels

### TODO
- [ ] Convert to Ackermann Steering
- [ ] Setup custom message for the serial data
- [ ] Warnings about battery voltage and current power consumption
