/* 
Class for data gathering from the arduino accelerometer.
*/

#ifndef GESTURE_RECOGNISER
#define GESTURE_RECOGNISER

#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <cmath>

#include "data_gatherer.h"
#include "model.h"

class GestureRecogniser: public DataGatherer {
    private:
      float SHOCK_W = 5.0; // Shock width
      float ACCEL_THRESHOLD = 1.0; // Acceleration tap threshold
      float TAP_SEP = 150.0; // Max double Tap separation
      void print_array(float *x, unsigned int n) {
        Serial.print("\n{");
        Serial.print(x[0]);
          for (int i=1; i<n; i++) {
            Serial.print(", ");
            Serial.print(x[i]);
          }
        Serial.print("}\n");
      }
      int K(float* x1, float* x2, unsigned int n, float gam=0.8) {
        float norm_squared = 0;
        for (int i=0; i<n; i++) {
          norm_squared = norm_squared + pow(x1[i] - x2[i], 2);
        }
        Serial.println(norm_squared);
        return exp(-gam*norm_squared);
      }
      int predict_gesture(float* X_test) {
        unsigned int N_s = sizeof(X) / sizeof(X[0]);
        unsigned int N_d = sizeof(X[0]) / sizeof(int);
        float results[3] = {0};
        float up_result, circle_result, left_result = 0;
        for (int i=0; i<N_s; i++) {
          float value = K(X[i], X_test, N_d);
          up_result = up_result + up_a[i]*up_t[i]*value;
          circle_result = circle_result + circle_a[i]*circle_t[i]*value;
          left_result = left_result + left_a[i]*left_t[i]*value;
          Serial.print("\nup_a[i]: ");
          Serial.print(up_a[i], 10);
          Serial.print("\nup_t[i]: ");
          Serial.print(up_t[i], 10);
          Serial.print("\nvalue: ");
          Serial.print(value, 10);
        }
        Serial.print("\nUp value: ");
        Serial.print(up_result, 10);
        Serial.print("\nCircle value: ");
        Serial.print(circle_result, 10);
        Serial.print("\nLeft value: ");
        Serial.print(left_result, 10);
        Serial.print("\nN_s: ");
        Serial.print(N_s);
        Serial.print("\nN_d: ");
        Serial.print(N_d);
        Serial.print("\n");

        // Choose maximum value
        int maxi=0;
        for (int i=0; i<3; i++) {if (results[maxi]<results[i]) {maxi=i;}}

        // Return 3 if no gesture matches
        if (results[maxi]<0) {return 3;}

        // Return number corresponding to gesture (0=up, 1=circle, 2=left)
        return maxi;
      }
    public:
      void detect_tap(int max_taps=1) {
        float t_prev = millis();
        float t_rise = millis();
        float t = millis()+10;
        bool risen_above = false;
        bool tap_detected = false;
        float x_prev = 0, y_prev = 0, z_prev = 0;
        float dx_prev = 0, dy_prev = 0, dz_prev = 0;
        int num_taps = 0;

        while (num_taps < max_taps) {

          float x, y, z, t;
          get_one(x, y, z, t);
          t = millis();
          float dydt = (y - y_prev) / (t - t_prev);
          float ddydt = (dydt - dy_prev) / (t - t_prev);

//          Serial.print(y);
//          Serial.print("dydt:");
//          Serial.print(10*dydt);
//          Serial.print("\t");
//          Serial.print("ddydt:");
//          Serial.println(20*ddydt);
//          Serial.print(t);
//          Serial.print("\n");

//          if (dydt > 0.3) {
//            Serial.println("Tap detected on derivative");
//          }
          if (ddydt > 0.05 && !risen_above) {
//            Serial.println("Tap detected on Second derivative");
            num_taps ++;
            risen_above = true;
          } else if (ddydt < 0.05) {
             risen_above = false;
          }

          dy_prev = dydt;
          x_prev = x; y_prev = y; z_prev = z;
          t_prev = t;
        }
      }

      int detect_gesture() {
        Serial.println("Detecting gesture...");
        int bins = 5;
        float features[39] = {0};
        float prev_accel[3] = {0};
        float prev_vel[3] = {0};
        float prev_pos[3] = {0};
        float prev_t = millis();
        
        reset_time();
        for (int i=0; i<100; i++) {
          // Get one point
          float x, y, z, t;
          get_one(x, y, z, t);
          float accel[3] = {x, y, z};
          if (prev_t == 0) {prev_t = t;}

          for (int i=0; i<3; i++) { // For each dimension

            // Get Velocity and Position
            float vel = prev_vel[i] + (accel[i] + prev_accel[i]) * (t - prev_t) / 2000;
            float pos = prev_pos[i] + (vel + prev_vel[i]) * (t - prev_t) / 2000;

             // Summation
            features[i] = features[i] + accel[i];
            features[i+3] = features[i+3] + vel;
            features[i+6] = features[i+6] + pos;

            // Velocity distribution
            int vel_dist_i=0;
            while (vel_dist_i+1 < bins*(vel - v_range[i][0]) / (v_range[i][1] - v_range[i][0])) {vel_dist_i+=1;}
            vel_dist_i = max(0, min(bins-1, vel_dist_i)); // Limit value
            features[i*bins+9+vel_dist_i] += 1;

            // Position distribution
            int pos_dist_i=0;
            while (pos_dist_i+1 < bins*(pos - pos_range[i][0]) / (pos_range[i][1] - pos_range[i][0])) {pos_dist_i+=1;}
            pos_dist_i = max(0, min(bins-1, pos_dist_i)); // Limit value
            features[i*bins+9+3*bins+pos_dist_i] += 1;

            // Max
//            if (accel[i] > features[i+9]) {features[i+9] = accel[i];}
//            if (vel > features[i+12]) {features[i+12] = vel;}
//            if (pos > features[i+15]) {features[i+15] = pos;}
//
//            // Min
//            if (accel[i] < features[i+18]) {features[i+18] = accel[i];}
//            if (vel < features[i+21]) {features[i+21] = vel;}
//            if (pos < features[i+24]) {features[i+24] = pos;}

            // Set prev values
            prev_accel[i] = accel[i];
            prev_vel[i] = vel;
            prev_pos[i] = pos;
          }
          prev_t = t;
        }
        Serial.println("Features found: ");
        print_array(features, 39);

        return predict_gesture(features);

        // Predict gesture
//        float w[27] = {-1.45879655e-01,  7.95745193e-02, -3.09944673e-01,  2.29617252e-01,
//                      -7.18024253e-02, -6.89768553e-02, -1.92722147e-01, -1.06457241e-01,
//                       2.00766313e-01,  8.87211409e-02, -3.27164233e-01,  4.21729810e-01,
//                       6.39833633e-03, -1.44722037e-02, -4.64916670e-03,  1.61956952e-02,
//                      -7.07734482e-03, -1.13653280e-03, -8.02804892e-02,  2.47649399e-01,
//                      -1.24674623e-01,  3.19285117e-03,  1.02109597e-02, -5.10140227e-04,
//                      -1.25069886e-02,  4.98022259e-03, -1.17672554e-05};
//        float b = 24.18716052;
//
//        int result = 0;
//        float y = b;
//        for (int i=0; i<27; i++) {
//          y = y + w[i] * features[i];
//        }
//
//        if (y > 0) {
//          result = 1;
//        }
//        return result;
      }
};



#endif // GESTURE_RECOGNISER
