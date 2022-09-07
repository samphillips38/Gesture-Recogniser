
#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <cmath>

//// Number of samples considered
int n_curr = 0;
//// How often we should save a measurement during downsampling
int sample_every_n;
//// The number of measurements since we last saved one
int sample_skip_counter = 1;
//// The expected accelerometer data sample frequency
const float kTargetHz = 100;
//// Total number of samples to consider
int n_samples = 0;
//// Shock width
float SHOCK_W = 10.0;
//// Acceleration tap threshold
float ACCEL_THRESHOLD = 0.04;
//// Max double Tap separation
float TAP_SEP = 150.0;

// Good parameters: SHOCK_W = 10.0, ACCEL_THRESHOLD = 2.0, TAP_SEP = 400.0

bool SetupAccelerometer() {
  // Switch on the IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialise IMU");
    return false;
  }

  // Make sure we are pulling measurements into a FIFO.
  // If you see an error on this line, make sure you have at least v1.1.0 of the
  // Arduino_LSM9DS1 library installed.
  IMU.setContinuousMode();

  // Determine how many measurements to keep in order to
  //  meet kTargetHz
  float sample_rate = IMU.accelerationSampleRate();
  sample_every_n = static_cast<int>(roundf(sample_rate / kTargetHz));

  Serial.println("Starting...");

  return true;
}

void ReadAccelerometer() {
  float t_prev = millis()+10;
  float t_prev_prev = millis();
  float y_prev = 0;
  float z_prev = 0;
  float y_prev_prev = 0;
  float z_prev_prev = 0;
  float t_prev_tap_y = millis();
  float t_prev_tap_z = millis();
  
  bool first_tap_y = false;
  bool first_tap_z = false;

  bool second_tap_y = false;
  bool second_tap_z = false;
  while (n_curr < n_samples) {
    while (IMU.accelerationAvailable()) {
      float x, y, z;
      // Read each sample, removing it from the device's FIFO buffer
      if (!IMU.readAcceleration(x, y, z)) {
        Serial.println("Failed to read data");
        break;
      }
      // Throw away this sample unless it's the nth
      if (sample_skip_counter != sample_every_n) {
        sample_skip_counter += 1;
        continue;
      }
   
      // Since we took a sample, reset the skip counter
      sample_skip_counter = 1;

      // Record time that data was gathered
      float t = millis();
      
      // Calculate the second derivative of acceleration
      float dt1 = t_prev - t_prev_prev;
      float dt2 = t - t_prev;
      float ddydt = 2*(y*dt1 + y_prev_prev*dt2 - y_prev*(dt1 + dt2)) / (dt1 * dt2 * (dt1+dt2));

      // Check if second derivative exceeds threshold and register tap
      if (ddydt > ACCEL_THRESHOLD) {
        if (t - t_prev_tap_y < TAP_SEP) { // This is the second tap
          Serial.println("Double Tap detected on y-axis!");
          second_tap_y = true;
          first_tap_y = false;
        } else { // This is the first tap
          first_tap_y = true;
          t_prev_tap_y = millis();
        }
      }

//      float ddzdt = 2*(z*dt1 + z_prev_prev*dt2 - z_prev*(dt1 + dt2)) / (dt1 * dt2 * (dt1+dt2));
//
//      // Check if second derivative exceeds threshold and register tap
//      if (ddzdt > ACCEL_THRESHOLD) {
//        if (t - t_prev_tap_z < TAP_SEP) { // This is the second tap
////          Serial.println("Double Tap detected on z-axis!");
//          second_tap_z = true;
//          first_tap_z = false;
//        } else { // This is the first tap
////          Serial.println("One Tap detected on z-axis!");
//          first_tap_z = true;
//          t_prev_tap_z = millis();
//        }
//      }

//      if (first_tap_y && !(first_tap_z && ddydt > ddzdt)) {
//        Serial.println("Tap detected on the y-axis!");
//      } else if (first_tap_z && !(first_tap_y && ddzdt > ddydt)) {
//        Serial.println("Tap detected on the z-axis!");
//      }
//
//      if (second_tap_y && !(second_tap_z && ddydt > ddzdt)) {
//        Serial.println("Double Tap detected on the y-axis!");
//      } else if (second_tap_z && !(second_tap_y && ddzdt > ddydt)) {
//        Serial.println("Double Tap detected on the z-axis!");
//      }

//      first_tap_y = false;
//      first_tap_z = false;

      else if (first_tap_y && t - t_prev_tap_y > TAP_SEP) { // Single tap detected
        Serial.println("One Tap detected on y-axis!");
        first_tap_y = false;
      }
        
      second_tap_y = false;
      second_tap_z = false;

      // Set data for next iteration
      t_prev_prev = t_prev;
      t_prev = t;
      
      y_prev_prev = y_prev;
      y_prev = y;
      z_prev_prev = z_prev;
      z_prev = z;
      
      n_curr++;
    }
  }
  n_curr = 0;
}

void setup() {
  Serial.begin(9600);
  SetupAccelerometer();
  return;
}

void loop() {
  n_samples = Serial.parseInt();
  if (n_samples > 0) {
    Serial.print("\nGathering ");
    Serial.print(n_samples);
    Serial.print(" samples \n");
    ReadAccelerometer();
    Serial.println("Finished Gathering Samples.\n");
  }
}
