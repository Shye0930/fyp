# Camera Calibration

This repository provides camera calibration tools for two different ROS versions: ROS2 Foxy and ROS2 Rolling.

## Folder Structure
- **ros2_foxy**: Contains calibration scripts and assets for ROS2 Foxy.
- **ros2_rolling**: Contains calibration scripts for ROS2 Rolling.

# ESP32-CAM LED Configuration

This document outlines the steps to configure the LED flash for the ESP32-CAM in the Arduino project.

## Modifying the LED Pin

To configure the LED flash, locate the `app_httpd.cpp` file in the Arduino project folder.

1. Open the `CameraWebServer.ino` file.
2. Navigate to the `startCameraServer()` function and access the `app_httpd.cpp` file from there.
3. In `app_httpd.cpp`, find the section labeled `// Enable LED Flash setup`.
4. Update the LED pin definition by changing:
   ```cpp
   #define LED_LEDC_GPIO 22 //configure LED pin
   ```
   to:
   ```cpp
   #define LED_LEDC_GPIO 4 //configure LED pin
   ```

## Enabling Independent LED Flash Control

To allow the LED flash to operate independently of the streaming state, modify the LED intensity logic in `app_httpd.cpp`.

1. Locate the following code block:
   ```cpp
   else if (!strcmp(variable, "led_intensity")) {
       led_duty = val;
       if (isStreaming) {
           enable_led(true);
       }
   }
   ```
2. Replace it with:
   ```cpp
   else if (!strcmp(variable, "led_intensity")) {
       led_duty = val;
       enable_led(true);
   }
   ```

This modification ensures the LED flash can be controlled regardless of whether the camera is streaming.


## Recommendation
It is **highly recommended** to use the calibration program in the `ros2_rolling` folder. The ROS2 Rolling version:
- Allows users to hold the calibration board further back, resulting in improved calibration accuracy.
- Includes optimizations for better performance.

Please refer to the respective `README.md` files in the `ros2_foxy` and `ros2_rolling` folders for detailed instructions on usage.