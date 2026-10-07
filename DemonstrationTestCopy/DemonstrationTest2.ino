#include <Wire.h>

#include "RobotWifiAP.h"

/** @brief WiFi access-point and TCP server used for optional telemetry. */
RobotWifiAP_c server("henry", "henryspassword", 80);

#include "Robot.h"
#include "RobotIMU.h"
#include "Controller.h"
#include "TaskTimer.h"

int led_brightness;

TaskTimer_c LED_timer;


/** @brief Hardware-facing interface to the Pololu robot. */
Robot_c robot;
/** @brief Direct interface to the 3Pi+ accelerometer, gyro, and magnetometer. */
RobotIMU_c imu;
/** @brief Supplied nominal line-following controller. */
Controller_c controller;


/** @brief Initialises serial, WiFi, IMU, and the supplied robot-controller baseline. */
void setup() {

  Serial.begin(115200);
  server.begin();

  robot.initialise(/*"your_teamname"*/);


  // Robot_c starts the shared I2C bus before RobotIMU_c uses it. IMU readings
  // come directly from the sensor ICs and do not pass through the middleware.
  if (!imu.initialise() ) {
    Serial.println("Warning: LSM6DS33 IMU was not detected.");
  }
  robot.setPose(0.0f, 0.0f, 0.0f);
  LED_timer.setIntervalMS(500);
  led_brightness = 0;


}

/** @brief Maintains telemetry transport and runs the controller repeatedly. */
void loop() {

  // Has the user pressed the button?
  bool button_state = robot.isButtonPressed();

  if (button_state == true) {
    // beep!
    robot.playTone(440, 50);
  }

  // Has timer finished counting?
  if (LED_timer.isReady()) {

    if (led_brightness == 0) {
      led_brightness = 100;
    } else {
      led_brightness = 0;
    }

    robot.setLED(255, 0, 0, led_brightness);

    LED_timer.resetTimer();
  }

}