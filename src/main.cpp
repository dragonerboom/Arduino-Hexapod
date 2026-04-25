#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <Ramp.h>

// ########################################## PHYSICAL CONSTANTS
const double L_COXA  = 57.0;  // J1L
const double L_FEMUR = 110.0; // J2L
const double L_TIBIA = 110.0; // J3L

const double Y_REST = 80.0;   // Neutral distance from body
const double Z_REST = -90.0;  // Standing height
const double TIBIA_OFFSET_ANGLE = 15.4; 

// ########################################## UNIFIED LEG CLASS
class UnifiedLeg {
  private:
    struct Joint {
      Servo servo;
      bool inverted;
      int8_t offset;
      void update(double angle) {
        double finalAngle = inverted ? (180 - angle + offset) : (angle + offset);
        servo.write(constrain(finalAngle, 0, 180));
      }
    };

    Joint j1, j2, j3;
    rampDouble posX, posY, posZ;
    double mountAngleRad;

  public:
    void setup(uint8_t p1, bool i1, int8_t o1, 
               uint8_t p2, bool i2, int8_t o2, 
               uint8_t p3, bool i3, int8_t o3, 
               float mAngleDeg) {
      
      j1.servo.attach(p1, 500, 2500); j1.inverted = i1; j1.offset = o1;
      j2.servo.attach(p2, 500, 2500); j2.inverted = i2; j2.offset = o2;
      j3.servo.attach(p3, 500, 2500); j3.inverted = i3; j3.offset = o3;
      
      mountAngleRad = mAngleDeg * (PI / 180.0);
      
      posX.go(0);
      posY.go(0);
      posZ.go(0);
    }

    void setTarget(double tx, double ty, double tz, uint32_t duration) {
      posX.go(tx, duration);
      posY.go(ty, duration);
      posZ.go(tz, duration);
    }

    bool isMoving() {
      return !posX.isFinished() || !posY.isFinished() || !posZ.isFinished();
    }

    void tick() {
      double curX = posX.update();
      double curY = posY.update();
      double curZ = posZ.update();

      // 1. HEXAGONAL TRANSFORMATION (Body Space -> Local Leg Space)
      double x = curX * cos(mountAngleRad) + curY * sin(mountAngleRad);
      double y = -curX * sin(mountAngleRad) + curY * cos(mountAngleRad);
      double z = curZ + Z_REST;
      y += Y_REST;

      // 2. INVERSE KINEMATICS
      double J1_A = atan2(x, y) * (180 / PI);
      double H = sqrt((y * y) + (x * x)) - L_COXA; 
      double L = sqrt((H * H) + (z * z));
      
      // Law of Cosines for J3
      double J3_A = acos(constrain(((L_FEMUR * L_FEMUR) + (L_TIBIA * L_TIBIA) - (L * L)) / (2 * L_FEMUR * L_TIBIA), -1, 1)) * (180 / PI);
      
      // Law of Cosines for J2
      double B = acos(constrain(((L * L) + (L_FEMUR * L_FEMUR) - (L_TIBIA * L_TIBIA)) / (2 * L * L_FEMUR), -1, 1)) * (180 / PI);
      double A = atan2(z, H) * (180 / PI);
      double J2_A = (B + A);

      // 3. SERVO UPDATES
      j1.update(90 - J1_A);
      j2.update(90 - J2_A);
      j3.update(j3.inverted ? (180 - (J3_A - TIBIA_OFFSET_ANGLE)) : (J3_A + TIBIA_OFFSET_ANGLE));
    }
};

// ########################################## GAIT SETTINGS
const double walkSteps[13][2] = {
    {0.0, 0.0}, {-0.2, 0.0}, {-0.6, 0.0}, {-1.0, 0.0},  // Stance
    {-0.8, 15.0}, {-0.4, 25.0}, {0.0, 30.0}, {0.4, 25.0}, // Swing
    {0.8, 15.0}, {1.0, 0.0}, {0.6, 0.0}, {0.3, 0.0}, {0.0, 0.0}
};

UnifiedLeg legs[6];
uint8_t stepA = 0, stepB = 6;
uint32_t walkSpeed = 80;

// User Input Variables
double stickMag = 0.0; 
double stickAngle = 0.0; // 0 = Forward, 180 = Backward
double rotation = 0.0; 

void setup() {
  // PIN ASSIGNMENTS (Fixed Pin 1 and Duplicate Pin 16 issues)
  // Format: (J1, Inv, Off, J2, Inv, Off, J3, Inv, Off, MountAngle)
  legs[0].setup(22, false, 0, 23, true,  0, 24, false, 0, 60);  // Front Right
  legs[1].setup(25, false, 0, 26, true,  0, 27, false, 0, 0);   // Mid Right
  legs[2].setup(28, false, 0, 29, true,  0, 30, false, 0, 300); // Back Right
  legs[3].setup(31, true,  0, 32, false, 0, 33, true,  0, 240); // Back Left
  legs[4].setup(34, true,  0, 35, false, 0, 36, true,  0, 180); // Mid Left
  legs[5].setup(37, true,  0, 38, false, 0, 39, true,  0, 120); // Front Left
  
  delay(2000);
}

void loop() {
  // 1. Refresh all leg IK and Servos
  for (int i = 0; i < 6; i++) legs[i].tick();

  // 2. Gait Sequence Trigger
  if (!legs[0].isMoving()) {
    // Only continue if stick is pushed OR we aren't back to neutral (0)
    if (stickMag > 0.1 || stepA != 0) {
      stepA = (stepA >= 12) ? 0 : stepA + 1;
      stepB = (stepB >= 12) ? 0 : stepB + 1;

      double rad = stickAngle * (PI / 180.0);
      double tX = stickMag * cos(rad);
      double tY = stickMag * sin(rad);

      for (int i = 0; i < 6; i++) {
        // Simple rotation: Add offset to Y based on rotation variable
        // (Outer legs move more in local space to create body rotation)
        double rotOffset = (i < 3) ? -rotation : rotation; 
        
        if (i % 2 == 0) // Group A
          legs[i].setTarget(tX * walkSteps[stepA][0], (tY + rotOffset) * walkSteps[stepA][0], walkSteps[stepA][1], walkSpeed);
        else            // Group B
          legs[i].setTarget(tX * walkSteps[stepB][0], (tY + rotOffset) * walkSteps[stepB][0], walkSteps[stepB][1], walkSpeed);
      }
    }
  }
}