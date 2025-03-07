//Includes ------------------------------------------------------------------------------------------------------------------------
#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>
#include <MadgwickAHRS.h>
#include <Pololu3piPlus32U4IMU.h>
using namespace Pololu3piPlus32U4;
OLED display;
Encoders encoders;
Buzzer buzzer;
Motors motors;
IMU imu;
Madgwick filter;
//Don't Touch ----------------------------------------------------------------------------------------------------------

float minX = -42.0, maxX = 36.0;
float minY = -50.0, maxY = 44.0;
float minZ = -55.0, maxZ = 49.0;

// Compute offsets
float offsetX = (maxX + minX) / 2.0;
float offsetY = (maxY + minY) / 2.0;
float offsetZ = (maxZ + minZ) / 2.0;

// Compute scale factors
float scaleX = (maxX - minX) / 2.0;
float scaleY = (maxY - minY) / 2.0;
float scaleZ = (maxZ - minZ) / 2.0;

float avgScale = (scaleX + scaleY + scaleZ) / 3.0;
float Sx = avgScale / scaleX;
float Sy = avgScale / scaleY;
float Sz = avgScale / scaleZ;
const float polling = 500; 

const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.26; 
const float BOT_RADIUS = 4.38; //cm horizontal radius(wheels to center)

// Global variables for gyro-based turning
volatile float ang = 0;
float ang_target;
int16_t  gyroRate      = 0;
int16_t  gyroOffset    = 0;
uint16_t gyroLastUpdate= 0;
bool cal = false;


// Misc. distances
float lt0, ls0, rt0, rs0;  
float start_time      = 0; 
float total_distance  = 0; 
volatile float eCount = 0;
volatile float eCount2= 0;
float total_turns     = 0;


// slow turns

const float pwm_min     = 38;     // minimal PWM for movement
float Kpt         = 0.75;    // proportional factor for turning
float left_angle  = 85.5;   // approx. “normal” left turn angle
float right_angle =  83.8;   // approx. “normal” right turn angle
const float turnTime = 900;
float full_turn=176;
float gyroBiasZ = 0.0;


//straight
const float kPs = 0.1;          // small angle correction for going straight
const float kP  = 0.4;            // for velocity control
const float str_min = 80;
const float mehta_sahni_constant = 0.042;

//Movement Values (Change here) ------------------------------------------------------------------------------------------------------------------------

float targetTime = 55;//subtract 1.75 seconds for every 50cm the end has, including the dist_diff  
//20 seconds for turn cali codes
float end_distance = 44.5; //
float end_delay = 0;
float time_diff=0;//seconds
float ang_diff=0;//left is positive
float dist_diff=0;//forwawrd is positive

//30.3 is enter
//for cali use 30 sec for 2 squares and 9 seconds for 4 backs
char movement[200] = "F29 R F50 L F200 R R F50 R F150 L F50 L F50 R F100 R F40 B F40 L F100 L F50 R F50 R F50 L F50 L F40 R R F40 R F50 L F100 R F100 R F50 R F40 R R F40 L F50 L F100 L F50 R E";
//F50 L F50 L F50 L F50 L F50 L F50 L F50 L F50 L;
//F50 R F50 R F50 R F50 R F50 R F50 R F50 R F50 R;
//F30.3 R F50 L F100 R F50 L F30 B F30 R F200 L F100 L F50 L F150 R F100 R F150 L F50 L F30 B F30 R F50 R F150 L E
//F50 B F50 B F50 B F50 B

//setup ================================================================================
void setup() {
  filter.begin(polling);
  delay(500);
  turnSensorSetup();
  
  calculateTotalTurns(movement);
  
  calculateTotalDistance(movement);
  start_time = micros();
  buzzer.playFrequency(440, 200, 15);
  buzzer.playFrequency(440, 200, 15);
  processCommands(movement);
  buzzer.playFrequency(440, 200, 15);
  buzzer.playFrequency(440, 200, 15);
 
  delay(5000);
}
//
void update() {
  eCount= encoders.getCountsLeft();
  eCount2 = encoders.getCountsRight();
  
}
void reset() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}
float dL(){return eCount / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE;}
float dR(){return eCount2 / (CLICKS_PER_ROTATION * GEAR_RATIO) * (WHEEL_CIRCUMFERENCE-mehta_sahni_constant);}
float vL(){
  float vel = (dL()-ls0)/(micros()-lt0)*1000000;
  ls0 = dL();
  lt0 = micros();
  return vel;
}
float vR(){
  float vel = (dR()-rs0)/(micros()-rt0)*1000000;  
  rs0 = dR();
  rt0 = micros();
  return vel;
}
void fwd(float distance) {
  update();
  float t0 = micros(); // Start time in microseconds
  float delta_T = findTime(distance);
  float delta_T_us = delta_T * 1e6; // Convert delta_T from seconds to microseconds
  float left_pwm = str_min;
  float right_pwm = str_min;
  
  float velocity_setpoint = 0; // Initialize the velocity setpoint
  float elapsed_time;

  // Main control loop
  while (true) {
    elapsed_time = micros() - t0; // Elapsed time in microseconds
    update();
    // Exit condition: Distance has been covered or time has exceeded delta_T
    if (dL() >= distance) {
      break;
    }

    // Determine velocity setpoint based on elapsed time
    if (elapsed_time <= delta_T_us / 4) {
      // Acceleration phase
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * (elapsed_time / 1e6);
    } else if (elapsed_time <= 3 * delta_T_us / 4) {
      // Constant velocity phase
      velocity_setpoint = (4.0 * distance) / (3.0 * delta_T);
    } else {
      // Deceleration phase
      float t_dec = elapsed_time - 3 * delta_T_us / 4;
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * ((delta_T / 4) - t_dec / 1e6);
    }

    // Update PWM values based on velocity feedback and setpoint
    float velocity_error_L = velocity_setpoint - vL();
    float velocity_error_R = velocity_setpoint - vR();

    left_pwm += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R;

    // Constrain PWM values to valid range
    left_pwm = constrain(left_pwm, str_min, 400);
    right_pwm = constrain(right_pwm, str_min, 400);
    right_pwm -= kPs * ang;
    // Set motor speeds
    motors.setSpeeds(left_pwm, right_pwm);
  }

  // Stop motors at the end
  motors.setSpeeds(0, 0);
  delay(10); // Small delay to ensure stop
  reset();   // Reset necessary parameters
}
void back(float distance) {
  delay(250);
  update();
  reset();
  while (ang < full_turn) {
    update();
    motors.setSpeeds(-pwm_min - abs(full_turn - (ang)) * Kpt/9, pwm_min + abs(full_turn - (ang)) * Kpt/9);
  }
  motors.setSpeeds(0, 0);
  delay(250);
  reset();
}

void end(float d) {
  motors.setSpeeds(0,0);
  reset();
  delay(200);
  float distance   = end_distance + d;
  float t0         = micros();
  float delta_T = (((targetTime *1e6 + start_time) - t0) / 1e6);
  float delta_T_us = delta_T * 1e6;
  if(findTime(distance) > delta_T){
    fwd(distance);
  }else{
    delta_T-=0.6;
    float left_pwm   = pwm_min;
    float right_pwm  = pwm_min;
    float velocity_setpoint = (distance) / (delta_T);
    float elapsed_time;
    
    while (true) {
      elapsed_time = micros() - t0;
      update();
      if (dL() >= distance) {
        break;
      }
      float velocity_error_L = velocity_setpoint - vL();
      float velocity_error_R = velocity_setpoint - vR();

      left_pwm  += kP * velocity_error_L;
      right_pwm += kP * velocity_error_R;
      left_pwm  = constrain(left_pwm,  pwm_min, 400);
      right_pwm = constrain(right_pwm, pwm_min, 400);
      right_pwm -= kPs * ang;

      motors.setSpeeds(left_pwm, right_pwm);
    }
  }
  motors.setSpeeds(0,0);
  if (ang_diff>0){
    delay(100);
    while (ang < ang_diff) {
      update();
      motors.setSpeeds(-pwm_min - abs(ang_diff - (ang)) * Kpt, pwm_min + abs(ang_diff - (ang)) * Kpt);
    }
  }else if (ang_diff<0){
    delay(100);
    while (ang > ang_diff) {  
      update();
      motors.setSpeeds(pwm_min + abs(-ang_diff + (ang)) * Kpt, -pwm_min - abs(-ang_diff + (ang)) * Kpt);
    }
  }
  fwd(dist_diff);
  display.clear();
  display.print(dL(),2);
  delay(500);
  display.clear();
  display.print((micros()-start_time)/1e6,2);
}

void left() {
  int starting = millis();
  delay(250);
  reset();
  while (ang < left_angle) {
    update();
    motors.setSpeeds(-pwm_min - abs(90 - (ang)) * Kpt, pwm_min + abs(90 - (ang)) * Kpt);
  }
  motors.setSpeeds(0, 0);
  delay(250);
  reset();
}


void right() {
  reset();
  int starting = millis();
  delay(250);
  while (ang > -right_angle) {
    update();
    motors.setSpeeds(pwm_min + abs(90 + (ang)) * Kpt, -pwm_min - abs(90 + (ang)) * Kpt);
  }
  motors.setSpeeds(0, 0);
  delay(250);
  reset();
}


void loop() {
  
  if(cal && micros() - 1/polling >=gyroLastUpdate){
    imu.readGyro();
    imu.readMag();
    float magX = imu.m.x - offsetX;
    float magY = imu.m.y - offsetY;
    float magZ = imu.m.z - offsetZ;
    magX *= Sx;
    magY *= Sy;
    magZ *= Sz;
    gyroLastUpdate = micros();
    filter.update(imu.g.x, imu.g.y,imu.g.z - gyroOffset, imu.a.x, imu.a.y, imu.g.z,  magX, magY, magZ);
    gyroLastUpdate = micros();
    ang = filter.getYaw();
  }
}

void turnSensorSetup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();
  display.clear();
  display.print(F("Gyro cal"));

  // Turn on the yellow LED in case the display is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) 
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;
  cal = true;
  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  
}
//helper functions -----------------------------------------------------------------------


float findTime(float distance){
  return (targetTime - total_turns*turnTime/1e3) * (distance/total_distance); //seconds
}
void calculateTotalDistance(const char* commands) {
  const char* ptr = commands;
  while (*ptr != '\0') {
    if (*ptr == 'F' || *ptr == 'B') {
      ptr++;
      float distance = 0.0;
      float factor = 1.0;
      bool isDecimal = false;
      while ((*ptr >= '0' && *ptr <= '9') || *ptr == '.') {
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
      total_distance += distance;
    } else if(*ptr ==' G'){
      total_distance += 70;
      
    } else{
      ptr++;
    }
  }
  total_distance += end_distance;
  total_distance -= total_turns * 2 * BOT_RADIUS;
}
void calculateTotalTurns(const char* commands) {
  const char* ptr = commands;
  while (*ptr != '\0') {
    if (*ptr == 'L' || *ptr == 'R') {
      total_turns++;
    }else if(*ptr =='B'){
      total_turns+=2;
    }
    ptr++;
  }
}


void processCommands(const char* commands) {
  const char* ptr = commands;
  bool previousBack=false;
  while (*ptr != '\0') {
    if (*ptr == 'F' || *ptr == 'B') {
      char cmd = *ptr; 
      ptr++; 
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
      if (cmd == 'F') fwd(distance);
        //previousBack=true;
      else back(distance);
    } else if (*ptr == 'B') {
        back(35);
        ptr++;
    } else if (*ptr == 'L') {
      left();
      ptr++;
    } else if (*ptr == 'R') {
      right();
      ptr++;
    } else if (*ptr == 'E') {
      end(0);
      ptr++;
    } else {
      ptr++; 
     
    }
  }
}



