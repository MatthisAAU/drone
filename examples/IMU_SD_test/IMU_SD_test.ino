// Standalone IMU + BMP280 + SD measurement for height estimation.
// No motor/receiver subsystem. Internal digital low-pass filters stay bypassed.
// Based on the IMU SD test sketch. Changes are marked with "NEW".
//
// NEW: uses HeightImuSdLogger (CSV, file HGTxxxx.CSV) instead of ImuSdLogger,
//      because ImuSdLogger has no fields for the barometer.
// NEW: BMP280 on I2C: VCC -> 3.3 V, GND -> GND, SDA -> pin 18, SCL -> pin 19.
//      Needs the "Adafruit BMP280 Library" (Library Manager, install all dependencies).
//
// Serial commands: S saves/stops (wait for closed=1), L starts a new file.
// Keep the board STILL and FLAT for the first 2 s of every recording.
#include <Wire.h>                 // NEW
#include <Adafruit_BMP280.h>      // NEW
#include <Sensors.h>
#include <Control.h>
#include "HeightImuSdLogger.h"    // NEW (instead of ImuSdLogger.h)

constexpr bool ESTIMATE_ATTITUDE = true;   // roll/pitch are needed for tilt compensation
constexpr uint32_t IMU_SPI_HZ = 1000000;
static_assert(IMU_SPI_HZ > 0 && IMU_SPI_HZ <= 7000000,
              "ICM-20948 SPI clock must be positive and at most 7 MHz.");
// NEW: 200 Hz instead of 1000 Hz. CSV formatting is slower than binary records,
// and 200 Hz is plenty for height estimation. You can try 500 if dropped stays 0.
constexpr uint32_t SAMPLE_RATE_HZ = 200;
static_assert(SAMPLE_RATE_HZ > 0 && SAMPLE_RATE_HZ <= 1000000,
              "Choose a positive acquisition rate up to 1000000 Hz.");
static_assert(1000000u % SAMPLE_RATE_HZ == 0,
              "Choose a rate with a whole-microsecond period.");
constexpr uint32_t SAMPLE_PERIOD_US = 1000000u / SAMPLE_RATE_HZ;
constexpr uint32_t BARO_PERIOD_US = 40000;   // NEW: barometer at 25 Hz

itc::Imu imu;
itc::AttitudeEstimator estimator;
itc::HeightImuSdLogger logger;   // NEW
Adafruit_BMP280 baro;            // NEW

uint32_t previousReadUs = 0;
uint32_t previousPrintMs = 0;
uint32_t windowReadings = 0;
uint32_t maximumGapUs = 0;
uint32_t previousSlotUs = 0;
uint32_t skippedSlots = 0;
uint32_t notReadyReads = 0;

uint32_t previousBaroUs = 0;     // NEW
float pressurePa = NAN;          // NEW
float temperatureC = NAN;        // NEW
bool baroNew = false;            // NEW

// ---------------------------------------------------------------------------
// NEW: ONLY PLACE THAT DEPENDS ON Sensors.h
// If this does not compile: open Sensors.h, look at "struct ImuSample" and put
// the names of the acceleration [g] and angular rate [deg/s] fields here.
// ---------------------------------------------------------------------------
void copyImu(const itc::ImuSample& s, itc::HeightImuRecord& r) {
  for (int i = 0; i < 3; ++i) {
    r.accelG[i]  = s.accelG[i];
    r.gyroDps[i] = s.gyroDps[i];
  }
}

void setup() {
  Serial.begin(itc::config::USB_BAUD);
  const uint32_t startedMs = millis();
  while (!Serial && uint32_t(millis() - startedMs) < 3000) delay(10);

  if (!imu.begin(true, IMU_SPI_HZ)) { // Keep digital filters bypassed.
    while (true) {
      Serial.print("IMU initialization failed at: ");
      Serial.println(imu.initializationStep());
      Serial.print("Driver status: ");
      Serial.println(imu.statusString());
      Serial.print("SPI clock Hz: ");
      Serial.println(IMU_SPI_HZ);
      delay(1000);
    }
  }

  // NEW: barometer. Board with only VCC/GND/SDA/SCL has a fixed address,
  // so both possible addresses are tried.
  Wire.begin();
  Wire.setClock(400000);
  if (!baro.begin(0x76) && !baro.begin(0x77)) {
    while (true) {
      Serial.println("BMP280 not found: check VCC=3.3V, GND, SDA=pin 18, SCL=pin 19");
      delay(1000);
    }
  }
  baro.setSampling(Adafruit_BMP280::MODE_NORMAL,
                   Adafruit_BMP280::SAMPLING_X1,   // temperature oversampling
                   Adafruit_BMP280::SAMPLING_X8,   // pressure oversampling
                   Adafruit_BMP280::FILTER_OFF,    // the Kalman filter does the filtering
                   Adafruit_BMP280::STANDBY_MS_1);

  logger.begin();   // NEW: opens HGTxxxx.CSV in service()
  Serial.println("IMU + BMP280 test: internal digital low-pass filters bypassed.");
  Serial.print("Target acquisition Hz: ");
  Serial.println(SAMPLE_RATE_HZ);
  Serial.println("Barometer: 25 Hz. Keep the board still and flat for the first 2 s.");
  Serial.println("S saves/stops; wait for closed=1. L starts a new file.");

  previousPrintMs = millis();
  previousSlotUs = micros();
  previousBaroUs = micros();   // NEW
}

void loop() {
  // Handle commands even when the sensor stops producing data.
  if (Serial.available()) {
    const int command = Serial.read();
    if (command == 'S') logger.requestStop();
    if (command == 'L') logger.requestStart();
  }

  const uint32_t nowUs = micros();

  // NEW: barometer at 25 Hz (skip missed periods, do not catch up)
  const uint32_t dueBaro = uint32_t(nowUs - previousBaroUs) / BARO_PERIOD_US;
  if (dueBaro != 0) {
    previousBaroUs += dueBaro * BARO_PERIOD_US;
    temperatureC = baro.readTemperature();
    pressurePa = baro.readPressure();   // [Pa]
    baroNew = true;
  }

  // Attempt one new IMU read per scheduled slot.
  const uint32_t dueSlots = uint32_t(nowUs - previousSlotUs) / SAMPLE_PERIOD_US;
  const bool sampleDue = dueSlots != 0;
  if (sampleDue) {
    previousSlotUs += dueSlots * SAMPLE_PERIOD_US;
    skippedSlots += dueSlots - 1;
  }

  itc::ImuSample measurement;
  if (sampleDue && imu.readFresh(measurement)) {
    const uint32_t readUs = micros(); // Host read completion, not sensor time.
    const uint32_t elapsedUs = uint32_t(readUs - previousReadUs);
    itc::Attitude attitude;
    if (ESTIMATE_ATTITUDE && previousReadUs != 0) {
      attitude = estimator.update(measurement, elapsedUs * 1e-6f);
    } else {
      attitude.rollDeg = attitude.pitchDeg = attitude.yawDeg = NAN;
    }
    if (previousReadUs != 0 && elapsedUs > maximumGapUs) maximumGapUs = elapsedUs;
    previousReadUs = readUs;
    ++windowReadings;

    // NEW: one CSV row = IMU + attitude + latest barometer value
    itc::HeightImuRecord record;
    record.timestampUs = readUs;
    copyImu(measurement, record);
    record.rollDeg = attitude.rollDeg;
    record.pitchDeg = attitude.pitchDeg;
    record.pressurePa = pressurePa;
    record.temperatureC = temperatureC;
    record.baroNew = baroNew ? 1 : 0;
    baroNew = false;   // flag only in the first row after a new baro value
    logger.sample(record);
  } else if (sampleDue) {
    ++notReadyReads; // No new reading OR a sensor/bus error; never copy the cache.
  }

  // SD can still stall this loop; the queue holds the samples meanwhile.
  logger.service(true); // Safe only because this sketch has no motors.

  // Report once a second, rather than printing every measurement.
  const uint32_t nowMs = millis();
  const uint32_t windowMs = uint32_t(nowMs - previousPrintMs);
  if (windowMs < 1000) return;
  char line[224];
  const int length = snprintf(line, sizeof(line),
      "frames_hz=%.1f max_gap_us=%lu skipped_slots=%lu no_read=%lu p=%.1fPa rec=%u closed=%u failed=%u queued=%u queue_dropped=%lu file=%s\n",
      windowReadings * 1000.0f / windowMs, (unsigned long)maximumGapUs,
      (unsigned long)skippedSlots, (unsigned long)notReadyReads, pressurePa,
      unsigned(logger.recording()), unsigned(logger.closed()), unsigned(logger.failed()),
      logger.queuedSamples(), (unsigned long)logger.droppedSamples(), logger.filename());
  if (Serial && length > 0 && length < int(sizeof(line)) && Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), length);
  }
  previousPrintMs = nowMs;
  windowReadings = maximumGapUs = 0;
  skippedSlots = notReadyReads = 0;
}