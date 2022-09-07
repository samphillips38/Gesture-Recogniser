/* 
Class for data gathering from the arduino accelerometer.
*/

#ifndef DATA_GATHER
#define DATA_GATHER

#include <Arduino.h>
#include <Arduino_LSM9DS1.h>

#include "circular_buffer.h"

class DataGatherer {
    private:
        int target_T = 8; // Target period in milliseconds
        unsigned long t0;
        CircularBuffer* last_second;
    public:
        DataGatherer() {
            // Switch on the IMU
            if (!IMU.begin()) {
                Serial.println("Failed to initialise IMU");
            }

            // Make sure we are pulling measurements into a FIFO.
            IMU.setContinuousMode();
            
            // Set initial time
            t0 = millis();

            // Create buffer
            int n = (1000 / target_T);
            last_second = new CircularBuffer(n);
        }

        void set_target_T(int val) {
          target_T = val;
        }

        void reset_time() {
          t0 = millis();
        }
        
        void get_one(float &x, float &y, float &z, float &t) {
          // Wait for time to be reached. Calculate average in the meantime
          unsigned long t1 = millis();
          int n_samples = 0;
          x = y = z = 0.0;
          while (t1 - t0 < target_T || n_samples == 0) {
              if (IMU.accelerationAvailable()) {
                  float xi, yi, zi;
                  // Read each sample, removing it from the device's FIFO buffer
                  if (!IMU.readAcceleration(xi, yi, zi)) {
                      Serial.println("Failed to read data");
                      break;
                  }
                  x = x + xi;
                  y = y + yi;
                  z = z + zi;
                  n_samples++;

                  t1 = millis();
              }
          }
//          Serial.println("Actual T: ");
//          Serial.print(t1 - t0);
//          Serial.print("\n");
          t = t1;
          x = x/n_samples;
          y = y/n_samples;
          z = z/n_samples;
          t0 = millis(); // Save time this sample was gathered
        }
        
        
        /// Get n samples approximately every T milliseconds
        CircularBuffer* get_samples_by_time(int n, bool print_output = true) {
            CircularBuffer* buffer_ptr = new CircularBuffer(n);

            int data_added = 0;
            while (data_added < n) {
                
                // Get single point
                float x, y, z, t;
                get_one(x, y, z, t);
                data_added++;

                // Print out data point if selected
                if (print_output) {
                    Serial.print("x:");
                    Serial.print(x);
                    Serial.print("\t");
                    Serial.print("y:");
                    Serial.print(y);
                    Serial.print("\t");
                    Serial.print("z:");
                    Serial.print(z);
                    Serial.print("\t");
                    Serial.print("t:");
                    Serial.println(t);
                }
            }
            return buffer_ptr;
        }
};



#endif // DATA_GATHER
