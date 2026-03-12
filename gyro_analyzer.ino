/*
 * Gyro Data Logger for Pololu 3Pi+ 32U4
 * ---------------------------------------
 * Logs gyro data over Serial as CSV for offline analysis.
 * Uses LSM6DS33 gyro, scale factor 0.07 deg/s per LSB.
 *
 * Usage:
 *   1. Upload to 3Pi+
 *   2. Keep robot STATIONARY
 *   3. Open Serial Monitor at 115200, let run 5 minutes
 *   4. Copy all output into gyro_log.csv
 *   5. Run: python gyro_analyzer.py gyro_log.csv
 *
 * Output CSV:
 *   timestamp_us, raw_rate_dps, corrected_rate_dps, heading_deg, dt_us
 */

#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4IMU.h>
using namespace Pololu3piPlus32U4;

OLED display;
IMU imu;

const double GYRO_SCALE = 0.07;  // deg/s per LSB

// Calibration
double gyroOffset = 0;
const int CAL_SAMPLES = 4096;

// State
double heading = 0.0;
unsigned long prevMicros = 0;
unsigned long startMicros = 0;
unsigned long sampleCount = 0;

// Stats
double sumRate = 0.0;
double sumRateSq = 0.0;

// Logging
const unsigned long LOG_DURATION_MS = 300000;  // 5 minutes
bool done = false;

void calibrateGyro() {
  display.clear();
  display.print(F("Gyro cal"));
  ledYellow(1);
  delay(500);

  double sum = 0;
  double sumSq = 0;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    while (!imu.gyroDataReady()) {}
    imu.readGyro();
    double raw = (double)imu.g.z;
    sum += raw;
    sumSq += raw * raw;
  }

  gyroOffset = sum / CAL_SAMPLES;
  double variance = (sumSq / CAL_SAMPLES) - (gyroOffset * gyroOffset);
  double stddev = sqrt(variance);

  ledYellow(0);

  display.clear();
  display.print(F("Off:"));
  display.print(gyroOffset * GYRO_SCALE, 4);
  delay(500);

  Serial.print("# Calibration complete. Offset: ");
  Serial.print(gyroOffset, 4);
  Serial.print(" LSB (");
  Serial.print(gyroOffset * GYRO_SCALE, 6);
  Serial.println(" deg/s)");
  Serial.print("# StdDev: ");
  Serial.print(stddev * GYRO_SCALE, 6);
  Serial.println(" deg/s");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  display.clear();
  display.print(F("Gyro Log"));
  delay(500);

  Serial.println("# Pololu 3Pi+ Gyro Data Logger");
  Serial.println("# Units: degrees/sec");
  Serial.println("#");

  calibrateGyro();

  Serial.println("#");
  Serial.println("timestamp_us,raw_rate,corrected_rate,heading_deg,dt_us");

  prevMicros = micros();
  startMicros = micros();

  display.clear();
  display.print(F("Logging"));
}

void loop() {
  if (done) return;

  unsigned long now = micros();

  // Check duration
  if ((now - startMicros) / 1000 > LOG_DURATION_MS) {
    double meanRate = sumRate / sampleCount;
    double meanRateSq = sumRateSq / sampleCount;
    double stddev = sqrt(meanRateSq - meanRate * meanRate);
    double duration = (now - startMicros) / 1e6;

    Serial.println("#");
    Serial.println("# === LOGGING COMPLETE ===");
    Serial.print("# Total samples: ");
    Serial.println(sampleCount);
    Serial.print("# Duration: ");
    Serial.print(duration, 2);
    Serial.println(" seconds");
    Serial.print("# Mean corrected rate: ");
    Serial.print(meanRate, 8);
    Serial.println(" deg/s");
    Serial.print("# StdDev corrected rate: ");
    Serial.print(stddev, 8);
    Serial.println(" deg/s");
    Serial.print("# Final heading drift: ");
    Serial.print(heading, 6);
    Serial.println(" degrees");
    Serial.print("# Drift rate: ");
    Serial.print(heading / duration, 6);
    Serial.println(" deg/s");
    Serial.print("# Suggested deadzone: ");
    Serial.print(stddev * 2.5, 8);
    Serial.println(" deg/s (2.5 sigma)");

    display.clear();
    display.print(F("Done"));
    display.gotoXY(0, 1);
    display.print(heading, 3);

    ledGreen(1);
    done = true;
    return;
  }

  // Wait for fresh gyro data (respects sensor ODR ~416Hz)
  if (!imu.gyroDataReady()) return;
  imu.readGyro();

  // Convert
  double rawRate = (double)imu.g.z * GYRO_SCALE;
  double correctedRate = ((double)imu.g.z - gyroOffset) * GYRO_SCALE;

  // dt
  unsigned long dtUs = now - prevMicros;
  prevMicros = now;
  double dt = dtUs / 1e6;

  // Integrate (no deadzone — we want raw drift data for analysis)
  heading += correctedRate * dt;

  // Stats
  sumRate += correctedRate;
  sumRateSq += correctedRate * correctedRate;
  sampleCount++;

  // CSV output
  Serial.print(now - startMicros);
  Serial.print(",");
  Serial.print(rawRate, 8);
  Serial.print(",");
  Serial.print(correctedRate, 8);
  Serial.print(",");
  Serial.print(heading, 6);
  Serial.print(",");
  Serial.println(dtUs);
}
