// Includes ------------------------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>
#include <PololuBuzzer.h>
#include <Pololu3piPlus32U4IMU.h>
using namespace Pololu3piPlus32U4;
OLED display;
Encoders encoders;
Buzzer buzzer;
Motors motors;
IMU imu;
// Don't Touch ----------------------------------------------------------------------------------------------------------
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.18;
const float BOT_RADIUS = 4.09; // cm horizontal radius(wheels to center)
double swerve_kP =  0; //0.05;
double swerve_kD = 2.5;//0.1;
double swerve_kA = 0;
// Global variables for gyro-based turning
double turnAngle = 0;
double turnRate = 0;
double gyroOffset = 0;
double gyroLastUpdate = 0;
double left_slip_factor = 1.0;
double right_slip_factor = 1.0;
double velocity_setpoint = 10.0; // cm/s

const unsigned long holdRequiredMs = 3000;
unsigned long tHighStart = 0;
const unsigned long debounceMs = 20;

// dillon new
double total_fwd = 0;
double delayer_amt;

// Misc. distances
double lt0, ls0, rt0, rs0;
double start_time = 0;
double total_distance = 0;
volatile double eCount = 0;
volatile double eCount2 = 0;
double total_turns = 0;

// slow turns

const double pwm_min = 80; // minimal PWM for movement
double Kpt = 0;            // proportional factor for turning
double left_angle = 34.15;  // approx. “normal” left turn angleseq
double right_angle = 34.15; // approx. “normal” right turn angle



double left_angleX = 72.5;  // approx. “normal” left turn angle
double right_angleX = 83; // approx. “normal” right turn angle
const double turnTime = 600;
double full_turn = 174;

// straight
double kPs = 0.5; // small angle correction for going straight
double kP = 0.4;  // for velocity control
double str_min = 30;
double mehta_sahni_constant = 0.045; // low to veer right, high to veer left

// Movement Values (Change here) ------------------------------------------------------------------------------------------------------------------------

double targetTime = 12; 

double lengthDist = 1000;
double offset = 90; 

double end_distance; //
double end_delay = 0;
double time_diff = 0; // seconds
double ang_diff = 0;  // left is positive
double dist_diff = 0; // forwawrd is positive

//F is forward movement (# after is cm)
//L and R are turns, if you dont put number it is 90 if you do then it is custom angle
String buildCanBypass()
{
  // totalDist = straight distance from A to B (cm)
  // offset = sideways distance from the center line to the can gap (cm)

  double halfDist = lengthDist / 2.0;
  double angleRad = atan(offset / halfDist); // radians
  double angleDeg = (angleRad * 180.0 / M_PI); // degrees

  // distance along the diagonal to midpoint offset
  double leg = sqrt(pow(halfDist, 2) + pow(offset, 2));

  String seq = "";  

  seq += "F400"; // String(lengthDist + BOT_RADIUS/2 - 2 * offset, 1) + " ";  // long straight
  end_distance=(offset - BOT_RADIUS/2) * sqrt(2);
  return seq;
}
char movement[200];


// setup ================================================================================
void setup()
{

  String moveSeq = buildCanBypass(); 
  moveSeq.toCharArray(movement, sizeof(movement));
  delay(1000);
  turnSensorSetup();
  turnSensorReset();
  calculateTotalTurns(movement);
  calculateTotalFwd(movement);
  delayer_amt = (time_diff * 1000) / (total_turns + total_fwd);
  calculateTotalDistance(movement);
  const uint8_t DN1_PIN = A11;  
  pinMode(DN1_PIN, INPUT);
  buzzer.playFrequency(880, 120, 15);


  bool armed = false;

  while (!armed)
  {
    int s = digitalRead(DN1_PIN);

    if (s == LOW)
    {
      if (tHighStart == 0) tHighStart = millis();

      // if it’s been high long enough, mark ready
      if (millis() - tHighStart >= holdRequiredMs)
      {
        armed = true;

        // play "ready to release" tone
        buzzer.playFrequency(1200, 200, 15);
        buzzer.playFrequency(1600, 200, 15);

        // wait until the trigger goes LOW to actually start moving
        while (digitalRead(DN1_PIN) == LOW) { delay(1); }
        delay(debounceMs);
      }
    }
    else
    {
      tHighStart = 0;
    }

  }


  start_time = micros();

  processCommands(movement);
  buzzer.playFrequency(440, 200, 15);
  buzzer.playFrequency(440, 200, 15);
  
  delay(5000);
}
//
void update()
{
  turnSensorUpdate();
  eCount += encoders.getCountsAndResetLeft();
  eCount2 += encoders.getCountsAndResetRight();
}
void reset()
{
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  eCount = 0;
  eCount2 = 0;
  turnSensorReset();
  turnAngle = 0;
}

double dL() { return eCount / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE; }
double dR() { return eCount2 / (CLICKS_PER_ROTATION * GEAR_RATIO) * (WHEEL_CIRCUMFERENCE - mehta_sahni_constant); }
double vL()
{
  double vel = (dL() - ls0) / (micros() - lt0) * 1000000;
  ls0 = dL();
  lt0 = micros();
  return vel;
}
double vR()
{
  double vel = (dR() - rs0) / (micros() - rt0) * 1000000;
  rs0 = dR();
  rt0 = micros();
  return vel;
}

void fwd(double distance)
{
  update();
  double t0 = micros(); // Start time in microseconds
  double delta_T = 2;
  double delta_T_us = delta_T * 1e6; // Convert delta_T from seconds to microseconds
  double left_pwm = str_min * 2;
  double right_pwm = str_min * 2 -4;

  double velocity_setpoint = 0; // Initialize the velocity setpoint
  double elapsed_time;

  // Main control loop
  while (true)
  {
    elapsed_time = micros() - t0; // Elapsed time in microseconds
    update();
    // Exit condition: Distance has been covered or time has exceeded delta_T
    if (dL() >= distance)
    {
      break;
    }
    velocity_setpoint = distance/delta_T;

    // Update PWM values based on velocity feedback and setpoint
    double velocity_error_L = velocity_setpoint - vL();
    double velocity_error_R = velocity_setpoint - vR();

    left_pwm += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R;

    // Constrain PWM values to valid range
    left_pwm = constrain(left_pwm, 0, 400);
    right_pwm = constrain(right_pwm, 0, 400);
    right_pwm -= kPs * ang();
    // Set motor speeds
    motors.setSpeeds(left_pwm, right_pwm);
  }

  // Stop motors at the end
  motors.setSpeeds(0, 0);
  delay(10 + delayer_amt); // Small delay to ensure stop
  reset();                 // Reset necessary parameters
}


void longf(double distance)
{
  update();
  double t0 = micros(); // Start time in microseconds
  double delta_T = targetTime - 5;
  double delta_T_us = delta_T * 1e6; // Convert delta_T from seconds to microseconds
  double left_pwm = str_min;
  double right_pwm = str_min;

  double velocity_setpoint = 0; // Initialize the velocity setpoint
  double elapsed_time;

  // Main control loop


  while (true)
  {
    elapsed_time = micros() - t0; // Elapsed time in microseconds
    update();
    // Exit condition: Distance has been covered or time has exceeded delta_T
    if (dL() >= distance)
    {
      break;
    }

    // Determine velocity setpoint based on elapsed time
    if (elapsed_time <= delta_T_us / 16)
    {
      // Acceleration phase
      velocity_setpoint = (256.0 * distance) / (15.0 * delta_T * delta_T) * (elapsed_time / 1e6);
    }
    if (elapsed_time <= 15 * delta_T_us / 16)
    {
      // Constant velocity phase
      velocity_setpoint = (16.0 * distance) / (14.0 * delta_T);
    }
    else
    {
      // Deceleration phase
      double t_dec = elapsed_time - 15 * delta_T_us / 16;
      velocity_setpoint = (256.0 * distance) / (15.0 * delta_T * delta_T) * ((delta_T / 16) - t_dec / 1e6);
    }

    // Update PWM values based on velocity feedback and setpoint
    double velocity_error_L = velocity_setpoint - vL();
    double velocity_error_R = velocity_setpoint - vR();

    left_pwm += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R;

    // Constrain PWM values to valid range
    left_pwm = constrain(left_pwm, 0, 400);
    right_pwm = constrain(right_pwm, 0, 400);
    right_pwm -= kPs * ang();
    // Set motor speeds
    motors.setSpeeds(left_pwm, right_pwm);
  }


  // Stop motors at the end
  motors.setSpeeds(0, 0);
  delay(10 + delayer_amt); // Small delay to ensure stop
  reset();                 // Reset necessary parameters

}

void end(double d)
{
  motors.setSpeeds(-10,-10);
  reset();
  delay(200);
  double distance = end_distance + d;
  double t0 = micros();
  double delta_T = (((targetTime * 1e6 + start_time) - t0) / 1e6);
  double delta_T_us = delta_T * 1e6;
  if (findTime(distance) > delta_T)
  {
    fwd(distance);
  }
  else
  {
    double left_pwm = 50;
    double right_pwm = 50 + 0.08;
    double velocity_setpoint = 2 * (distance) / (delta_T);
    double elapsed_time;

    while (true)
    {
      elapsed_time = micros() - t0;
      update();
      if (dL() >= distance)
      {
        break;
      }
      double velocity_error_L = velocity_setpoint - vL();
      double velocity_error_R = velocity_setpoint - vR();

      left_pwm += kP * velocity_error_L;
      right_pwm += kP * velocity_error_R;
      left_pwm = constrain(left_pwm, 35, 400);
      right_pwm = constrain(right_pwm, 35, 400);
      right_pwm -= kPs * ang();

      motors.setSpeeds(left_pwm, right_pwm);
    }
  }
  motors.setSpeeds(-10,-10);
  if (ang_diff > 0)
  {
    delay(100);
    while (ang() < ang_diff)
    {
      update();
      motors.setSpeeds(-pwm_min - abs(ang_diff - (ang())) * Kpt, pwm_min + abs(ang_diff - (ang())) * Kpt);
    }
  }
  else if (ang_diff < 0)
  {
    delay(100);
    while (ang() > ang_diff)
    {
      update();
      motors.setSpeeds(pwm_min + abs(-ang_diff + (ang())) * Kpt, -pwm_min - abs(-ang_diff + (ang())) * Kpt);
    }
  }
  fwd(dist_diff);
  delay(500);

}



void left()
{
  int starting = millis();
  update();
  delay(200);
  while(fabs(ang()) < left_angle)//while (-dL() + dR() < BOT_RADIUS * 3.14159)
  {
    update();
    motors.setSpeeds(-pwm_min -4, pwm_min);
  }
  motors.setSpeeds(-5,-5);
  // delay(turnTime > (millis() - starting) ? turnTime - (millis() - starting) + 150: 150);
  delay(200);
  reset();
}


void pivotLeft(double angleDeg)
{
  reset();
  update();
  delay(50);

  double right_pwm = 100;
  double velocity_error_R;
  double angleRad = angleDeg * M_PI / 180.0;

  // arc length for right wheel to pivot left about left wheel
  double arcLength = 2.0 * BOT_RADIUS * angleRad;

  while (dR() < arcLength)
  {
    update();
    velocity_error_R = vR() - velocity_setpoint;
    right_pwm -= swerve_kD * velocity_error_R;
    motors.setSpeeds(0, right_pwm);
  }

  motors.setSpeeds(0, 0);
  delay(50);
  reset();
}
void left(int val) {
  int starting = millis();
  delay(150);
  reset();
  int32_t total = 0;

  for (uint16_t i = 0; i < 64; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();


    // Add the Z axis reading to the total.
    total += imu.g.z;
  }

  gyroOffset = total / 64;
  while (ang() < val*(left_angleX/90)) {
    update();
    motors.setSpeeds(-pwm_min - abs(val - (ang())) * Kpt, pwm_min + abs(val - (ang())) * Kpt);
  }
  motors.setSpeeds(0, 0);
  
  delay(250);
  reset();
}
void pivotRight(double angleDeg)
{
  reset();
  update();
  delay(200);

  // constants
  double left_pwm = 100;
  double velocity_error_L;
  double angleRad = angleDeg * M_PI / 180.0;

  // arc length the moving wheel should travel
  double arcLength = 2.0 * BOT_RADIUS * angleRad;  // right wheel pivot

  while (dL() < arcLength)
  {
    update();
    velocity_error_L = vL() - velocity_setpoint;
    left_pwm -= swerve_kD * velocity_error_L;
    motors.setSpeeds(left_pwm, 0);
  }

  motors.setSpeeds(0, 0);
  delay(100);
  reset();
}
void right()
{
  int starting = millis();
  update();
  delay(200);
  while(fabs(ang()) < right_angle)//while (dL() + -dR() < BOT_RADIUS * 3.14159)
  {
    update();
    motors.setSpeeds(pwm_min+4, -pwm_min);
  }
  motors.setSpeeds(-5,-5);
  // delay(turnTime > (millis() - starting) ? turnTime - (millis() - starting) + 150: 150);
  delay(200);
  reset();
}


void right(int val) {
  reset();
  int starting = millis();
  delay(150);
  reset();
  int32_t total = 0;
  for (uint16_t i = 0; i < 64; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();


    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  gyroOffset = total / 64;
  while (ang() > -val*(right_angleX/90)) {
    update();
    motors.setSpeeds(pwm_min + abs(val + (ang())) * Kpt, -pwm_min - abs(val + (ang())) * Kpt);
  }
  motors.setSpeeds(0, 0);
  delay(250);
  reset();
}

// helper functions -----------------------------------------------------------------------

void loop()
{
}
double findTime(double distance)
{
  return (targetTime - total_turns * (turnTime) / 1e3) * (distance / total_distance); // seconds
}
void calculateTotalDistance(const char *commands)
{
  total_distance = 2 * offset + BOT_RADIUS + lengthDist;
}
void calculateTotalTurns(const char *commands)
{
  const char *ptr = commands;
  while (*ptr != '\0')
  {
    if (*ptr == 'L' || *ptr == 'R')
    {
      total_turns++;
    }
    else if (*ptr == 'B')
    {
      total_turns += 2;
    }
    ptr++;
  }
}
void calculateTotalFwd(const char *commands)
{
  const char *ptr = commands;
  while (*ptr != '\0')
  {
    if (*ptr == 'F')
    {
      total_fwd++;
    }
    ptr++;
  }
}

void processCommands(const char* commands) {
  const char* ptr = commands; // Pointer to traverse the char array
   // Flag to track if the previous command was a turn
  while (*ptr != '\0') { // Loop until null terminator
    if (*ptr == 'F') {
      char cmd = *ptr; // Store the command ('F' or 'B')
      ptr++; // Move to the number part
      float distance = 0.0;
      float factor = 1.0;
      bool isDecimal = false;
      while ((*ptr >= '0' && *ptr <= '9') || *ptr == '.') { // Extract number
        if (*ptr == '.') {
          isDecimal = true;
          factor = 0.1;
        } else if (!isDecimal) {
          distance = distance * 10 + (*ptr - '0');
        } else {
          distance += (*ptr - '0') * factor;
          factor *= 0.1;
        }
        ptr++;
      }


      if (cmd == 'F') {
        if(distance<150){
          fwd(distance);
        }else{
          longf(distance);
        }
      }
    } else if (*ptr == 'L') {
      ptr++;
      float turn_angs = 0.0;
      float factor = 1.0;
      bool isDecimal = false;
      while ((*ptr >= '0' && *ptr <= '9') || *ptr == '.') { // Extract number
        if (*ptr == '.') {
          isDecimal = true;
          factor = 0.1;
        } else if (!isDecimal) {
          turn_angs = turn_angs * 10 + (*ptr - '0');
        } else {
          turn_angs += (*ptr - '0') * factor;
          factor *= 0.1;
        }
        ptr++;
      }
      if (turn_angs==0){
        left();
      }else{
        pivotLeft(turn_angs);
      }
      ptr++;

    } else if (*ptr == 'R') {
      ptr++;
      float turn_angs = 0.0;
      float factor = 1.0;
      bool isDecimal = false;
      while ((*ptr >= '0' && *ptr <= '9') || *ptr == '.') { // Extract number
        if (*ptr == '.') {
          isDecimal = true;
          factor = 0.1;
        } else if (!isDecimal) {
          turn_angs = turn_angs * 10 + (*ptr - '0');
        } else {
          turn_angs += (*ptr - '0') * factor;
          factor *= 0.1;
        }
        ptr++;
      }
      if (turn_angs==0){
        right();
      }else{
        pivotRight(turn_angs);
      }
      ptr++;
    } else if (*ptr == 'E') {

        end(0);//-BOT_RADIUS

      ptr++;
    } else if (*ptr == 'D') {
      ptr++;
      float delaytime = 0.0;
      float factor = 1.0;
      bool isDecimal = false;
      while ((*ptr >= '0' && *ptr <= '9') || *ptr == '.') { // Extract number
        if (*ptr == '.') {
          isDecimal = true;
          factor = 0.1;
        } else if (!isDecimal) {
          delaytime = delaytime * 10 + (*ptr - '0');
        } else {
          delaytime += (*ptr - '0') * factor;
          factor *= 0.1;
        }
        ptr++;
      }
      if (delaytime==0){
        delay(500);
      }else{
        delay(delaytime);
      }
      ptr++;
    } else {
      ptr++; // Skip unrecognized characters
     
    }
  }
}
//Pololu included gyro stuff


// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}


// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate()
{
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;
  if(fabs(turnRate) * 0.07 < 0.02){return;}

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;
  
  double d = turnRate * 0.07 * dt/1e6;

  turnAngle += d;
}
/*
int32_t ang(){ return ((((int32_t)turnAngle >> 16) * 360) >> 16);
  }
*/
double ang()
{
  return turnAngle;
}
void turnSensorSetup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();


  // Turn on the yellow LED in case the display is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady())
    {
    }
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.

  turnSensorReset();

  turnSensorUpdate();

}
