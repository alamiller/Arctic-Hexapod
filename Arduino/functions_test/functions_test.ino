// test custom functions
// The purpose of this script is to verify the implementation of new
// functions to the

#ifndef ARDUINO_AVR_MEGA2560
#error Wrong board. Please choose "Arduino/Genuino Mega or Mega 2560"
#endif

#include <FNHR.h>

FNHR robot;

void setup() {
  // Start Freenove Hexapod Robot with default function
  robot.Start(true);
  Serial.begin(9600);
}

double mapservos(double servoAngle) {
  /*
     Maps the the desired angle input for a given robot joint
     and outputs the necessary angle command to have legs 3-4
     move in the same direction as legs 1-3 for a given angle
     command.
  */
  double hexapodAngle;
  hexapodAngle = 180 - servoAngle;
  return hexapodAngle;
}

void test(int input) {
  int i;
  int j;
  int k;

  for (i = 1; i <= 6; i++) {
    robot.RotateJoints(i, 90, 90, 90);
  }

  switch (input) {
    case 1:
      // test hip
      Serial.println("TESTNG HIP JOINTS");
      
      // Tests the hip in the backwards direction
      for (i = 1; i <= 6; ++i) {
        for (j = 90; j >= 30; j = j - 10) {
          Serial.print(i); Serial.print(":"); Serial.println(j); Serial.println(" Degrees");
          if (i <= 3 ) {
            robot.RotateJoints(i, j, 90, 90);
          } else {
            robot.RotateJoints(i, mapservos(j), 90 , 90);
          }
          
          delay(1000);
          
        }
        robot.RotateJoints(i, 90, 90, 90);
        delay(1000);
      }
      // Tests the hip in the forward direction
      for (i = 1; i <= 6; ++i) {
        for (j = 90; j <= 170; j = j + 10) {
          Serial.print(i); Serial.print(":"); Serial.println(j); Serial.println(" Degrees");
          robot.RotateJoints(i, j, 90, 90);
          delay(1000);
        }
        robot.RotateJoints(i, 90, 90, 90);
        delay(500);
      }
      break;
    case 2:
      // test knee
      Serial.println("TESTNG KNEE JOINTS");


      for (i = 1; i <= 6; ++i) {
        for (j = 90; j >= -10; j = j - 10) {
          
          Serial.print(i); Serial.print(":"); Serial.println(j); Serial.println(" Degrees");
          
          if (i <= 3 ) {
            robot.RotateJoints(i, 90, j, 90);
          } else {
            robot.RotateJoints(i, 90, mapservos(j), 90);
          }
          
          delay(1000);
          
        }
        
        robot.RotateJoints(i, 90, 90, 90);
        delay(1000);
        
      }
      
      for (i = 1; i <= 6; ++i) {
        for (j = 90; j <= 150; j = j + 10) {
          
          Serial.print(i); Serial.print(":"); Serial.println(j); Serial.println(" Degrees");
          
          if (i <= 3 ) {
            robot.RotateJoints(i, 90, j, 90);
          } else {
            robot.RotateJoints(i, 90, mapservos(j), 90);
          }
          
          delay(1000);
          
        }
        
        robot.RotateJoints(i, 90, 90, 90);
        delay(1000);
        
      }
      break;

    case 3:
      Serial.println("TESTNG ANKLE JOINTS");
      // Rotate tibia inwards
      for (i = 1; i <= 6; ++i) {
        for (j = 90; j >= 20; j = j - 10) {
          Serial.print(i); Serial.print(":"); Serial.println(j); Serial.println(" Degrees");
          if (i <= 3 ) {
            robot.RotateJoints(i, 90, 90, j);
          } else {
            robot.RotateJoints(i, 90, 90, mapservos(j));
          }
          delay(1000);
        }
        robot.RotateJoints(i, 90, 90, 90);
        delay(500);
      }
      // Rotate tibia outwards
      for (i = 1; i <= 6; ++i) {
        for (j = 90; j <= 210; j = j + 10) {
          Serial.print(i); Serial.print(":"); Serial.println(j); Serial.println(" Degrees");
          if (i <= 3 ) {
            robot.RotateJoints(i, 90, 90, j);
          } else {
            robot.RotateJoints(i, 90, 90, mapservos(j));
          }
          delay(1000);
        }
        robot.RotateJoints(i, 90, 90, 90);
        delay(500);
      }
      break;
    default:
      // code block
      Serial.println("TEST NOT SPECIFIED");
  }
  Serial.println("TEST ENDED");
  robot.SleepMode();
  delay(1000);
  robot.ActiveMode();
  delay(1000);
}

void loop() {
  // Update Freenove Hexapod Robot
  robot.Update();

  while (true) {
    test(2);
  }

  while (true) {
    robot.RotateJoints(1, 90, 90, 90);
    robot.RotateJoints(2, 90, 90, 90);
    robot.RotateJoints(3, 90, 90, 90);
    robot.RotateJoints(4, 90, 90, 90);
    robot.RotateJoints(5, 90, 90, 90);
    robot.RotateJoints(6, 90, 90, 90);

    int i;
    int j;
    int k;

    Serial.println("TESTNG HIP JOINTS");

    // test hip
    for (i = 1; i < 7; ++i) {
      for (j = 90; j >= 45; j = j - 10) {
        Serial.print(i); Serial.print(":"); Serial.println(j); Serial.println(" Degrees");

        if (i <= 3 ) {
          robot.RotateJoints(i, j, 90, 90);
          delay(1000);
        } else
          robot.RotateJoints(i, mapservos(j), 90 , 90);
      }
      robot.RotateJoints(i, 90, 90, 90);
      delay(500);
    }
    for (i = 1; i < 7; ++i) {
      for (j = 90; j <= 180; j = j + 10) {
        Serial.print(i); Serial.print(":"); Serial.println(j); Serial.println(" Degrees");

        robot.RotateJoints(i, j, 90, 90);
        delay(1000);
      }
      robot.RotateJoints(i, 90, 90, 90);
      delay(500);
    }
    robot.SleepMode();
    delay(500);
    robot.ActiveMode();
    delay(100);

    robot.RotateJoints(1, 90, 90, 90);
    robot.RotateJoints(2, 90, 90, 90);
    robot.RotateJoints(3, 90, 90, 90);
    robot.RotateJoints(4, 90, 90, 90);
    robot.RotateJoints(5, 90, 90, 90);
    robot.RotateJoints(6, 90, 90, 90);


    Serial.println("TESTNG KNEE JOINTS");

    // test knee
    for (i = 1; i <= 6; ++i) {
      for (j = 90; j >= -10; j = j - 10) {
        Serial.print(i); Serial.print(":"); Serial.println(j); Serial.println(" Degrees");
        if (i <= 3 ) {
          robot.RotateJoints(i, 90, j, 90);
          delay(1000);
        } else
          robot.RotateJoints(i, 90, mapservos(j), 90);
      }
      robot.RotateJoints(i, 90, 90, 90);
      delay(500);
    }

    robot.SleepMode();
    delay(500);
    robot.ActiveMode();
    delay(100);

    robot.RotateJoints(1, 90, 90, 90);
    robot.RotateJoints(2, 90, 90, 90);
    robot.RotateJoints(3, 90, 90, 90);
    robot.RotateJoints(4, 90, 90, 90);
    robot.RotateJoints(5, 90, 90, 90);
    robot.RotateJoints(6, 90, 90, 90);

    Serial.println("TESTNG ANKLE JOINTS");
    for (i = 1; i < 7; ++i) {
      for (j = 90; j >= 20; j = j - 10) {
        Serial.print(i); Serial.print(":"); Serial.println(j); Serial.println(" Degrees");
        robot.RotateJoints(i, 90, 90, j);
        delay(1000);
      }
      robot.RotateJoints(i, 90, 90, 90);
      delay(500);
    }
    robot.SleepMode();
    delay(500);
    robot.ActiveMode();
    delay(100);


  }
}
