#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <cmath>

#include "data_gatherer.h"
#include "circular_buffer.h"
#include "gesture_recogniser.h"

// LED pins
#define RED 22     
#define BLUE 24     
#define GREEN 23

DataGatherer* dg_ptr;
GestureRecogniser* gr_ptr;

void setup() {
    Serial.begin(9600);
    pinMode(RED, OUTPUT);
    pinMode(BLUE, OUTPUT);
    pinMode(GREEN, OUTPUT);
    digitalWrite(RED, HIGH);
    digitalWrite(BLUE, HIGH);
    digitalWrite(GREEN, HIGH);
    dg_ptr = new DataGatherer();
    gr_ptr = new GestureRecogniser();
    return;
}

void loop() {
    if (Serial.available() > 0) {
        int byteInput = Serial.read();

        switch (byteInput)
        {
        case 'a':
            {
              digitalWrite(BLUE, LOW);
//              Serial.println("Detecting tap...");
              gr_ptr->detect_tap();
              // Do something
//              Serial.println("Function Finished");
              digitalWrite(BLUE, HIGH);
            }
            break;
        case 'b':
            {
              digitalWrite(RED, LOW);
                // Get samples using alternate method

                // Choose number of samples
//                Serial.println("Choose Number of Samples: ");
                int n = 0;
                while (n <= 0) {n = Serial.parseInt();}

                // Choose sample period
//                Serial.println("Choose Sample Period in Milliseconds: ");
                unsigned long T = 0;
                while (T <= 0) {T = Serial.parseInt();}
                dg_ptr->set_target_T(T);

//                 Serial.println("Running...");
                
                // Get samples
                CircularBuffer* buff_ptr;
                buff_ptr = dg_ptr->get_samples_by_time(n);

                digitalWrite(RED, HIGH);
            }
            break;
        case 'c':
            {
              dg_ptr->set_target_T(15);
              for (int i; i < 25; i++) {
                digitalWrite(GREEN, LOW);
                gr_ptr->detect_tap();
                digitalWrite(GREEN, HIGH);

                // Detect for around 1.5 seconds
                digitalWrite(BLUE, LOW);
                dg_ptr->get_samples_by_time(100);
                digitalWrite(BLUE, HIGH);

                Serial.println("");
              }
            }
            break;
        case 'd':
            {
              digitalWrite(GREEN, LOW);
              digitalWrite(BLUE, LOW);
              gr_ptr->set_target_T(15);
              int done = 0;
              GestureRecogniser* tap_detect_ptr = new GestureRecogniser();
              while (done != 'e') {
                if (Serial.available() > 0) {
                  done = Serial.read();
                }
                
                digitalWrite(BLUE, HIGH);
                tap_detect_ptr->detect_tap();
                digitalWrite(BLUE, LOW);
                int gesture = gr_ptr->detect_gesture();
                
                if (gesture==0) {
                  Serial.println("Circle");
                } else if (gesture==1) {
                  Serial.println("Up");
                } else if (gesture==2) {
                  Serial.println("Left");
                } else if (gesture==3) {
                  Serial.println("No gesture Found");
                } else {
                  Serial.println("Error");
                }
                
                
              }
              digitalWrite(GREEN, HIGH);
              digitalWrite(BLUE, HIGH);
            }
            break;
         
        default:
            break;
        }
    }
  
}
