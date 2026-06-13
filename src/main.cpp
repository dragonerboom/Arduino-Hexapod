#include <Arduino.h>
#include <math.h>
#include <Ramp.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ########################################## PHYSICAL CONSTANTS
const double L_COXA  = 57.0;  // J1L
const double L_FEMUR = 110.0; // J2L
const double L_TIBIA = 110.0; // J3L

const double Y_REST = 80.0;   // Neutral distance from body
const double Z_REST = -90.0;  // Standing height
const int ServoMin = 500;
const int ServoMax = 2000;

//##################################CLASSES########################
class UnifiedLeg {
  private:
    struct Joint {
      Adafruit_PWMServoDriver* driver; // Pointer to assigned driver board
      uint8_t channel;                 // driver pin channel (0-15)
      bool inverted;                   
      int8_t offset;                   

      void update(double angle) {
        double finalAngle = inverted ? (180.0 - angle + offset) : (angle + offset);
        finalAngle = constrain(finalAngle, 0.0, 180.0);

        // Convert target angle directly to microsecond pulse limits
        int pulseWidthMicros = map(finalAngle, 0.0, 180.0, ServoMin, ServoMax);
        
        driver->writeMicroseconds(channel, pulseWidthMicros);
      }
    };

    Joint j1, j2, j3;
    rampDouble posX, posY, posZ; 
    double mountAngleRad;

  public:
    void setup(Adafruit_PWMServoDriver &board, 
               uint8_t ch1, bool i1, int8_t o1, 
               uint8_t ch2, bool i2, int8_t o2, 
               uint8_t ch3, bool i3, int8_t o3, 
               float mAngleDeg) {
      
      // Map physical channels and orientations
      j1.driver = &board; j1.channel = ch1; j1.inverted = i1; j1.offset = o1;
      j2.driver = &board; j2.channel = ch2; j2.inverted = i2; j2.offset = o2;
      j3.driver = &board; j3.channel = ch3; j3.inverted = i3; j3.offset = o3;
      
      mountAngleRad = mAngleDeg * (PI / 180.0);
      
      posX.go(0);
      posY.go(0);
      posZ.go(0);
      delay(100);
    }

    void setTarget(double tx, double ty, double tz, uint32_t duration) {
      posX.go(tx, duration);
      posY.go(ty, duration);
      posZ.go(tz, duration);
    }

    bool isNotMoving() {
      return posX.isFinished() && posY.isFinished() && posZ.isFinished();
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

      // 2. INVERSE KINEMATICS (Pure Geometric Solves)
      double J1_A = atan2(x, y) * (180.0 / PI);
      double H = sqrt((y * y) + (x * x)) - L_COXA; 
      double L = sqrt((H * H) + (z * z));
      
      // Law of Cosines for J3 (Tibia angle relative to Femur)
      double J3_A = acos(constrain(((L_FEMUR * L_FEMUR) + (L_TIBIA * L_TIBIA) - (L * L)) / (2.0 * L_FEMUR * L_TIBIA), -1.0, 1.0)) * (180.0 / PI);
      
      // Law of Cosines for J2 (Femur angle relative to Horizon)
      double B = acos(constrain(((L * L) + (L_FEMUR * L_FEMUR) - (L_TIBIA * L_TIBIA)) / (2.0 * L * L_FEMUR), -1.0, 1.0)) * (180.0 / PI);
      double A = atan2(z, H) * (180.0 / PI);
      double J2_A = (B + A);

      // 3. HARDWARE POSITIONING
      // Joint 1 (Coxa): Centered at 90 deg, sweeps relative to Z-axis entry
      j1.update(90.0 - J1_A);
      
      // Joint 2 (Femur): Centered at 90 deg relative to horizontal datum
      j2.update(90.0 - J2_A);
      
      // Joint 3 (Tibia): Cleaned of spatial offsets, maps pure physical closing angle
      j3.update(J3_A);
    }
};
UnifiedLeg legs[6];
Adafruit_PWMServoDriver boardLeft = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver boardRight = Adafruit_PWMServoDriver(0x41);
// ########################################## GAIT SETTINGS
const double walkSteps[13][2] = {
    {0.0, 0.0}, 
    {-0.2, 0.0}, 
    {-0.6, 0.0}, 
    {-1.0, 0.0},  // Stance
    {-0.8, 15.0}, 
    {-0.4, 25.0}, 
    {0.0, 30.0}, 
    {0.4, 25.0}, // Swing
    {0.8, 15.0}, 
    {1.0, 0.0}, 
    {0.6, 0.0}, 
    {0.3, 0.0}, 
    {0.0, 0.0}
};
//Variables
bool allStationary;
uint8_t stepA = 0, stepB = 6;
uint32_t walkSpeed = 80;

// User Input Variables
double stickMag = 0.0; 
double stickAngle = 0.0; // 0 = Forward, 180 = Backward
double rotation = 0.0; 

void setup() {
  boardLeft.begin();
  boardLeft.setPWMFreq(50);
  boardRight.begin();
  boardRight.setPWMFreq(50);
  delay(2000);
}

void loop() {
  // 1. Refresh all leg IK and Servos
  for (int i = 0; i < 6; i++) legs[i].tick();

  // 2. Gait Sequence Trigger

  allStationary = true;
  for (int i = 0; i < 6; i++) {
    if (!legs[i].isNotMoving()) {
        allStationary = false;
        break; // Stop checking immediately if one leg is moving
    }
  }

  if (allStationary) {
    // Only continue if stick is pushed OR we aren't back to neutral (0)
    if (stickMag > 0.1 || stepA != 0) {
      stepA = (stepA >= 12) ? 0 : stepA + 1;
      stepB = (stepB >= 12) ? 0 : stepB + 1;

      double rad = stickAngle * (PI / 180.0);
      double tX = stickMag * cos(rad);
      double tY = stickMag * sin(rad);

      for (int i = 0; i < 6; i++) {
        // Simple rotation: Add offset to Y based on rotation variable
        double rotOffset = (i < 3) ? -rotation : rotation; 
        /*   0        1
              ↖️-+-↗️
           2--⬅️-+-➡️--3
              ↙️-+-↘️
             4       5
        */
        //Group A
        legs[0].setTarget(tX * walkSteps[stepA][0], (tY + rotOffset) * walkSteps[stepA][0], walkSteps[stepA][1], walkSpeed);
        legs[3].setTarget(tX * walkSteps[stepA][0], (tY + rotOffset) * walkSteps[stepA][0], walkSteps[stepA][1], walkSpeed);
        legs[4].setTarget(tX * walkSteps[stepA][0], (tY + rotOffset) * walkSteps[stepA][0], walkSteps[stepA][1], walkSpeed);
        //Group B
        legs[1].setTarget(tX * walkSteps[stepB][0], (tY + rotOffset) * walkSteps[stepB][0], walkSteps[stepB][1], walkSpeed);
        legs[2].setTarget(tX * walkSteps[stepB][0], (tY + rotOffset) * walkSteps[stepB][0], walkSteps[stepB][1], walkSpeed);
        legs[5].setTarget(tX * walkSteps[stepB][0], (tY + rotOffset) * walkSteps[stepB][0], walkSteps[stepB][1], walkSpeed);
        /*
        if (i % 2 == 0) // Group A
          legs[i].setTarget(tX * walkSteps[stepA][0], (tY + rotOffset) * walkSteps[stepA][0], walkSteps[stepA][1], walkSpeed);
        else            // Group B
          legs[i].setTarget(tX * walkSteps[stepB][0], (tY + rotOffset) * walkSteps[stepB][0], walkSteps[stepB][1], walkSpeed);
        */
      }
    }
  }
}