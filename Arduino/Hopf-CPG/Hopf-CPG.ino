
#include <math.h>
#include <FNHR.h>

using namespace std;

#ifndef ARDUINO_AVR_MEGA2560
#error Wrong board. Please choose "Arduino/Genuino Mega or Mega 2560"
#endif

// Constants constraints for the hip, knee and ankle joint angles, respectedly.
#define THETA_ONE_LOWER 40
#define THETA_ONE_UPPER 140

#define THETA_TWO_LOWER 0
#define THETA_TWO_UPPER 140

#define THETA_THREE_LOWER 20
#define THETA_THREE_UPPER 180

FNHR robot;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  robot.Start();
}

double mapservos(double servoAngle) {
  /*
   * Translates the angles for legs 4-5 of the Freenove 
   * hexapod robot.
   */
  double hexapodAngle;
  hexapodAngle = 180 - servoAngle;
  return hexapodAngle;
}

class HexCPG {

  private:
    double alpha;
    double coupling_strength;
  public:
    // CPG parameters
    double beta = 1;
    double mu = 1;
    double x[6], y[6];
    double x_dot[6], y_dot[6];
    double omega = 3.14;

    // Phases to change walking gait
    double phase[6] = {180, 0, 180, 0, 180, 0}; // tripod
//    double phase[6] = {0, 180, 90, 0, 180, 270}; // quad gait

//    double phase[6] = {180, 90, 0, 0, 202.5, 180}; // ripple
//    double phase[6] = {0, 0, 0, 0, 0, 0}; // wave
    double timer;
    double sample_time = 0.5;

//    float k[5] = {60.0, 30, 30, -0.5, 0.5};
//    float b[5] = {90.0, 60.0, 40.0, 90.0, 90.0};
    float k[5] = {90.0, 30, 30, 0.7, -0.7};
//    float k[5] = {90.0, 30, 30, -0.0, 0.0};
    float b[5] = {90.0, 60.0, 40.0, 90.0, 90.0};

    double  theta1[6], theta2[6], theta3[6];

    // Methods
    void stand(float contact_angle) {
      // stand the robot
      float newang;
      newang = mapservos(110);

      for (int i = 0; i < 6; i++) {

        if (i < 3) {
          robot.RotateJoints(i + 1, 90, 90, 110);
        } else {
          robot.RotateJoints(i + 1, 90, 90, mapservos(contact_angle));
        }
      }
    }

    void setGait() {
      /*
       * Changes the gait of the robot by setting the swing phase of each leg.
      */
    }


    void walk() {
      double timer = 0;
      Serial.print("theta 12, "); Serial.print("joint 22, "); Serial.println("leg 32");
      while (true) {

        int i;
        for (i = 0; i < 6; i++) {
          y[i] = cos(omega * timer + phase[i] * (180 / PI));
          x[i] = sin(omega * timer + phase[i] * (180 / PI) );
        }
        Serial.print(y[1]); Serial.print(" "); Serial.println(x[1]);

        // Calculate state derivates of oscillator
        for (i = 0; i < 6; i++) {
          x_dot[i] = alpha * ( mu - sq(x[i]) - sq(y[i])  ) * x[i] - omega * y[i];
          y_dot[i] = beta  * ( mu - sq(x[i]) - sq(y[i])  ) * y[i] + omega * x[i];
        }

        for (i = 0; i < 6; i++) {

          theta1[i] = k[0] * y[i] + b[0];

          if (y_dot[i] >= 0) {
            theta2[i] = k[1] * x[i] + b[1];
          } else {
            theta2[i] = k[2] * x[i] + b[2];
          }

          if (y_dot[i] >= 0) {
            theta3[i] = k[3] * theta2[i] + b[3];
          } else {
            theta3[i] = k[4] * theta2[i] + b[4];
          }
        }

        // remap the angles for legs 4-6
        for (i = 3; i < 6; i++) {
          theta1[i] = mapservos(theta1[i]);
          theta2[i] = mapservos(theta2[i]);
          theta3[i] = mapservos(theta3[i]);
        }

        // constrain the angles for the hardware of the robot
        // to prevent unintential movements of limbs
        for (i = 0; i < 6; i++) {
          theta1[i] = constrain(theta1[i], THETA_ONE_LOWER, THETA_ONE_UPPER);
          theta2[i] = constrain(theta2[i], THETA_TWO_LOWER, THETA_TWO_UPPER);
          theta3[i] = constrain(theta3[i], THETA_THREE_LOWER, THETA_THREE_UPPER );
        }

        Serial.print(theta1[1]); Serial.print(", "); Serial.print(theta2[2]); Serial.print(", "); Serial.println(theta3[3]);
        //    Serial.print("leg 4,"); Serial.print("joint 4, "); Serial.print(theta2[4]);
        //    Serial.print("leg 5,"); Serial.print("joint 5, "); Serial.print(theta2[5]);
        //    Serial.print("leg 6,"); Serial.print("joint 6, "); Serial.println(theta2[6]);

        // Send joint angle commands to the robot
        for (i = 1; i <= 6; i++) {
          robot.RotateJoints(i, theta1[i - 1], theta2[i - 1], theta3[i - 1]);

        }
        delay(sample_time * 1000);
        timer += sample_time;
      }
    }

};

// define all the CPG parameters
//double phase[6] = {90, 45, 0, 0, 270, 180}; // circling

void loop() {
  // put your main code here, to run repeatedly:

  HexCPG snowbot;

  // faste set-up
//  snowbot.omega = 30;
//  snowbot.sample_time = 0.03;
//  snowbot.k[3] = 0; snowbot.k[4] = 0;

  //slower
  snowbot.omega = 3.14;
  snowbot.sample_time = 0.5;
//  snowbot.k[3] = 0; snowbot.k[4] = 0;

  snowbot.stand(110);
  delay(4000);

  snowbot.walk();
}
