#include "Controller.h"

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;
const unsigned long Controller_c::RECOVERY_TIMEOUT_MS;
const uint16_t Controller_c::LINE_THRESHOLD;
constexpr float Controller_c::BASE_PWM;
constexpr float Controller_c::LINE_FOLLOW_GAIN;
constexpr float Controller_c::RECOVERY_LEFT_PWM;
constexpr float Controller_c::RECOVERY_RIGHT_PWM;

bool offset_test = true;

Controller_c::Controller_c()
  : update_timer(UPDATE_INTERVAL_MS),
    telemetry_timer(TELEMETRY_INTERVAL_MS) {
  reset();
}

void Controller_c::reset() {
  signal = 0;
  mode = WAITING;
  recovery_start_ms = 0;
  last_surface_us = 0;
  sample_num = 0;
  trial_num = 0;

}

uint8_t Controller_c::getSignal() const {
  return signal;
}

void Controller_c::setSignal(uint8_t requested_signal) {
  if (requested_signal == 0) {
    reset();
    return;
  }

  if (signal == 0) {
    signal = 1;
    mode = FOLLOWING_LINE;
    recovery_start_ms = 0;
  }
}

void Controller_c::update(Robot_c &robot, RobotWifiAP_c &server) {

  // Get the current count of milliseconds since the StampC3 was powered on.
  unsigned long now = robot.getMillis();

  // Check if our TaskTimer_c for the general controller update indicates
  // that it is the correct time to operate.  Note, we give the TaskTimer_c
  // the current count in milliseconds.
  // We do this because we don't want to query the robot faster than at 10ms
  // intervals.
  if (!update_timer.isReady(now)) {

    // TaskTimer_c says it is not ready, so we simply end this call to
    // the controller here by calling return.
    return;
  }

  // The TaskTimer_c was ready, so we now reset the TaskTimer_c so it
  // begins counting again.  We now run the remaining controller code.
  update_timer.resetTimer(now);

  // Ask the robot for latest information on surface reflectance sensors.
  bool surface_ok = robot.getSurfaceSensors();

  // Ask the robot for the latest encoder information, and then use this
  // to update the odometry on board the StampC3 (see Odometry.h)
  if (robot.getEncoders()) {
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  }

  // Pose estimation from the Pololu 3Pi robot itself.
  robot.getPose();

  // Monitor the button on the StampC3.  If the user presses it, we change
  // the signal variable to 1.  This is used to activate the Digital Twin
  // simulation demo.
if (robot.isButtonPressed()) {

    if (signal == 0) {
        signal = 1;
        mode = FOLLOWING_LINE;      // Start in line-following mode
        recovery_start_ms = 0;      // Reset recovery timer
        trial_num++;
        sample_num = 0;
    } else {
        signal = 0;
        mode = WAITING;             // Stop line following
        robot.setMotorPWM(0, 0);    // Stop the motors
    }
}

// Check if the robot has been activated by the button.
if (signal == 1) {

    // Check that we successfully received a new set of
    // surface sensor readings before running the test.
    if (surface_ok == true) {

        // Choose which behaviour to run.
        // If offset_test is true, collect and print sensor data.
        if (offset_test) {
            testOffset(robot, now);

        // If offset_test is false, run the normal line follower.
        } else {
            runLineFollower(robot, now);
        }

    }
}


// Check whether it is time to send another set of telemetry data.
// telemetry_timer controls how frequently this happens.
if (telemetry_timer.isReady(now)) {

  // Reset the timer so that we wait for the next interval
  // before sending telemetry again.
  telemetry_timer.resetTimer(now);

  // Send the current robot data over WiFi and Serial.
 // publishTelemetry(robot, server, now);
}

}


bool Controller_c::lineDetected(const Robot_c &robot) const {

  // Look at surface sensor number 2.
  // If its reading is greater than or equal to the LINE_THRESHOLD,
  // we consider a line to have been detected.
  if (robot.surface.reading[2] >= LINE_THRESHOLD) {

    // A line has been detected.
    return true;
  }

  // The sensor reading was below the threshold,
  // so we consider that there is no line.
  return false;
}


void Controller_c::runLineFollower(Robot_c &robot, unsigned long now) {

  // Ask lineDetected() whether the robot can currently see the line.
  // The result will be either true or false.
  bool line_detected = lineDetected(robot);


  // If the robot has lost the line while it was previously following it...
  if (!line_detected && mode == FOLLOWING_LINE) {

    // Change the robot's mode to searching for the line.
    mode = SEARCHING_FOR_LINE;

    // Record the time at which the robot lost the line.
    // This allows us to give the robot a limited amount of time
    // to find the line again.
    recovery_start_ms = now;
  }


  // Check whether the robot is currently searching for the line.
  if (mode == SEARCHING_FOR_LINE) {

    // If the line has been found again...
    if (line_detected) {

      // Return to normal line-following mode.
      mode = FOLLOWING_LINE;

    // Otherwise, check whether the recovery time has expired.
    } else if (now - recovery_start_ms >= RECOVERY_TIMEOUT_MS) {

      // The robot has searched for long enough without finding the line,
      // so change to STOPPED mode.
      mode = STOPPED;
    }
  }


  // If the robot is stopped...
  if (mode == STOPPED) {

    // Set both motors to 0, meaning the robot does not move.
    robot.setMotorPWM(0, 0);

    // Stop running the rest of this function.
    return;
  }


  // If the robot is searching for the line...
  if (mode == SEARCHING_FOR_LINE) {

    // Set the motors to the recovery speeds.
    // These values make the robot move in a way that searches for the line.
    robot.setMotorPWM(RECOVERY_LEFT_PWM, RECOVERY_RIGHT_PWM);

    // Stop running the rest of this function.
    return;
  }


  // Calculate the difference between the left and right sensor readings.
  // This tells us which direction the robot needs to turn.
  //
  // reading[1] = one side of the sensor array
  // reading[3] = the other side
  float error = (float)robot.surface.reading[1]
              - (float)robot.surface.reading[3];


  // Convert the sensor error into a steering correction.
  // A larger error produces a larger turn.
  float turn = error * LINE_FOLLOW_GAIN;


  // Set the motor speeds.
  //
  // One motor is slowed down and the other is sped up depending
  // on the value of 'turn', causing the robot to steer towards the line.
  robot.setMotorPWM(BASE_PWM - turn, BASE_PWM + turn);


}


void Controller_c::testOffset(Robot_c &robot, unsigned long now) {

    // Is this actually new sensor data?
    if (robot.surface.timestamp_us <= last_surface_us) {
        return;
    }

    // Remember this timestamp
    last_surface_us = robot.surface.timestamp_us;

    // This is a new sample
    sample_num++;

    // Print the five sensors as five CSV rows
    for (int sensor_num = 0; sensor_num < 5; sensor_num++) {

        Serial.print("offsetTest_white");
        Serial.print(",");

        Serial.print(trial_num);
        Serial.print(",");

        Serial.print(sample_num);
        Serial.print(",");

        Serial.print(robot.surface.timestamp_us);
        Serial.print(",");

        Serial.print(sensor_num);
        Serial.print(",");

        Serial.println(robot.surface.reading[sensor_num]);
    }

    robot.setMotorPWM(0, 0);
}

void Controller_c::publishTelemetry(
  Robot_c &robot,
  RobotWifiAP_c &server,
  unsigned long timestamp_ms
) {

  // Send the robot's current data over WiFi.
  //
  // The format string determines the type of each value:
  // %lu = unsigned long
  // %.2f = floating-point number with 2 decimal places
  // %.5f = floating-point number with 5 decimal places
  // %d = integer
  // %ld = long integer
  // %u = unsigned integer
  //
  // Each comma separates one piece of data.
  server.printf(
    "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",

    // Time since the program started.
    timestamp_ms,

    // Robot position and orientation.
    odometry.pose.x,
    odometry.pose.y,
    odometry.pose.theta,

    // Current left and right motor PWM values.
    robot.getLeftMotorPWM(),
    robot.getRightMotorPWM(),

    // Current left and right encoder counts.
    (long)robot.getLeftEncoderCount(),
    (long)robot.getRightEncoderCount(),

    // Readings from the five surface sensors.
    robot.surface.reading[0],
    robot.surface.reading[1],
    robot.surface.reading[2],
    robot.surface.reading[3],
    robot.surface.reading[4],

    // Current signal/mode value.
    signal
  );


  // Check whether the robot is connected to a Serial device.
  if (Serial) {

    // If Serial is connected, send exactly the same telemetry data
    // through the USB/Serial connection as well.
    Serial.printf(
      "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",

      // Time.
      timestamp_ms,

      // Position and orientation.
      odometry.pose.x,
      odometry.pose.y,
      odometry.pose.theta,

      // Motor speeds.
      robot.getLeftMotorPWM(),
      robot.getRightMotorPWM(),

      // Encoder counts.
      (long)robot.getLeftEncoderCount(),
      (long)robot.getRightEncoderCount(),

      // Five surface sensor readings.
      robot.surface.reading[0],
      robot.surface.reading[1],
      robot.surface.reading[2],
      robot.surface.reading[3],
      robot.surface.reading[4],

      // Signal value.
      signal
    );
  }
}