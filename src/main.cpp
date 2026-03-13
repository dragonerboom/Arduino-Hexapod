#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <Ramp.h>

// ########################################## CONSTANTS
const double J2L = 57.0; 
const double J3L = 110.0;
const double Y_Rest = 80.0;
const double Z_Rest = -90.0;
const double J3_LegAngle = 15.4;

//GAIT ARRAY (2 Columns: [0]=Swing Multiplier, [1]=Lift Height)
const double walk[13][2] = {
    {0.0,  0.0},  // 0: Neutral
    {-0.2, 0.0},  // 1: Stance
    {-0.6, 0.0},  // 2: Stance
    {-1.0, 0.0},  // 3: End Stance
    {-0.8, 15.0}, // 4: Lift Start
    {-0.4, 25.0}, // 5: Mid Swing
    {0.0,  30.0}, // 6: Peak
    {0.4,  25.0}, // 7: Mid Swing
    {0.8,  15.0}, // 8: Descent
    {1.0,  0.0},  // 9: Touchdown
    {0.6,  0.0},  // 10: Stance
    {0.3,  0.0},  // 11: Stance
    {0.0,  0.0}   // 12: Neutral
};

// ########################################## UNIFIED LEG CLASS
class UnifiedLeg {
  private:
    struct InternalJoint {
      Servo servo;
      bool inverted;
      int8_t offset;
      void setup(uint8_t pin, bool inv, int8_t off) {
        inverted = inv; offset = off;
        servo.attach(pin, 500, 2500);
      }
      void update(double angle) {
        if (inverted) servo.write(180 - angle + offset);
        else servo.write(angle + offset);
      }
    };

    InternalJoint j1, j2, j3;
    double _mountingAngle;

  public:
    void setup(uint8_t p1, bool i1, int8_t o1, uint8_t p2, bool i2, int8_t o2, uint8_t p3, bool i3, int8_t o3) {
      j1.setup(p1, i1, o1); j2.setup(p2, i2, o2); j3.setup(p3, i3, o3);
    }

    void cartesianMove(double X, double Y, double Z) {
      Y += Y_Rest;
      Z += Z_Rest;

      

      // Inverse Kinematics Solution
      double J1_A = atan(X / Y) * (180 / PI);
      double H = sqrt((Y * Y) + (X * X));
      double L = sqrt((H * H) + (Z * Z));
      double J3_A = acos(((J2L * J2L) + (J3L * J3L) - (L * L)) / (2 * J2L * J3L)) * (180 / PI);
      double B = acos(((L * L) + (J2L * J2L) - (J3L * J3L)) / (2 * L * J2L)) * (180 / PI);
      double A = atan(Z / H) * (180 / PI);
      double J2_A = (B + A);

      j1.update(90 - J1_A);
      j2.update(90 - J2_A);
      j3.update(j3.inverted ? (J3_A - J3_LegAngle) : (J3_A + J3_LegAngle));
    }
};


// ########################################## GLOBAL INSTANCES
UnifiedLeg legs[6];
rampDouble AXTar;
rampDouble AYTar;
rampDouble AZTar;

rampDouble BXTar;
rampDouble BYTar;
rampDouble BZTar;
// User Inputs
double stickMagnitude = 0.0; 
double stickAngle = 0.0;     // 0 = Forward, 90 = Right, 180 = Backward (Direction)
double rotationSpeed = 0.0; // Positive = Clockwise spin  
uint8_t walkSpeed = 75;      // Timing (ms per step frame)
double Zlevel;
bool started = false;
uint8_t stepA = 12, stepB = 6;

void setup() {
  // CONFIGURATION FOR RECTANGULAR CHASSIS
  
  // Format: (J1_Pin, J1_Inv, J1_Off, J2_Pin, J2_Inv, J2_Off, J3_Pin, J3_Inv, J3_Off)
  legs[0].setup(1,     false,       0,     2,   true,      0,      3,  false,      0); // Front Left
  legs[1].setup(4,     false,       0,     5,   true,      6,     25,  false,      0); // Middle Left
  legs[2].setup(7,     false,       0,     8,   true,      0,      9,  false,      0); // Back Left
  legs[3].setup(10,     true,       0,    23,  false,      0,     16,   true,      0); // Front Right
  legs[4].setup(17,     true,       0,    18,  false,      0,     19,  false,      0); // Middle Right
  legs[5].setup(15,     true,       0,     4,  false,      0,     16,   true,      0); // Back Right

  delay(2000);
}

void MotionControl() {
  // 1. Update Ramps
  double ax = AXTar.update(), ay = AYTar.update(), az = AZTar.update();
  double bx = BXTar.update(), by = BYTar.update(), bz = BZTar.update();

  // 2. Apply to 6 Legs
  for (int i = 0; i < 6; i++) {
    double x_trans, y_trans, z_lift;
    
    // Assign Tripod Group A or B
    if (i % 2 == 0) { x_trans = ax; y_trans = ay; z_lift = az; }
    else            { x_trans = bx; y_trans = by; z_lift = bz; }

    // --- RECTANGULAR ROTATION VECTORS ---
    double rotX = 0, rotY = 0;

    if (i == 0 || i == 2 || i == 4) { // LEFT SIDE (Even indices)
      rotX = rotationSpeed; 
      if (i == 0) rotY = rotationSpeed;  // Front Left Out
      if (i == 4) rotY = -rotationSpeed; // Back Left In
    } else {                         // RIGHT SIDE (Odd indices)
      rotX = -rotationSpeed; 
      if (i == 1) rotY = rotationSpeed;  // Front Right Out
      if (i == 5) rotY = -rotationSpeed; // Back Right In
    }

    // Normalized Swing Progress (-1 to 1) for Syncing Rotation
    // We add 1 to magnitude to avoid division by zero
    double swing = (i % 2 == 0) ? (ax / (stickMagnitude + 1)) : (bx / (stickMagnitude + 1));
    
    legs[i].cartesianMove(x_trans + (rotX * swing), y_trans + (rotY * swing),z_lift + Zlevel);
  }

  // 3. Logic to trigger next step frame
  if (!started || AXTar.isFinished()) {
    stepA = (stepA >= 12) ? 1 : stepA + 1;
    stepB = (stepB >= 12) ? 1 : stepB + 1;

    double rad = stickAngle * (PI / 180.0);
    double targetX = stickMagnitude * cos(rad);
    double targetY = stickMagnitude * sin(rad);

    // Group A 
    AXTar.go(walk[stepA][0] * targetX, walkSpeed);
    AYTar.go(walk[stepA][0] * targetY, walkSpeed);
    AZTar.go(walk[stepA][1],           walkSpeed);

    // Group B
    BXTar.go(walk[stepB][0] * targetX, walkSpeed);
    BYTar.go(walk[stepB][0] * targetY, walkSpeed);
    BZTar.go(walk[stepB][1],           walkSpeed);

    started = true;
  }
}

void loop() {
  MotionControl();
}