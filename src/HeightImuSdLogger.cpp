#include "HeightImuSdLogger.h"
#include <Arduino.h>
#include <stdio.h>

namespace itc {

void HeightImuSdLogger::begin() {
  requestStart(); // Actual card access waits until service() is allowed.
}

void HeightImuSdLogger::requestStart() {
  if (!fileOpen_ && !failed_) startRequested_ = true;
}

void HeightImuSdLogger::requestStop() {
  startRequested_ = false;
  recording_ = false;
  stopping_ = fileOpen_;
}

void HeightImuSdLogger::sample(const HeightImuRecord& reading) {
  if (!recording_) return;
  const uint32_t sequence = sequence_++;
  if (count_ == CAPACITY) {
    ++dropped_;
    return; // Do not wait for storage or overwrite a queued observation.
  }
  HeightImuRecord& record = records_[head_];
  record = reading;
  record.sequence = sequence;
  head_ = (head_ + 1) % CAPACITY;
  ++count_;
}

bool HeightImuSdLogger::openNewFile() {
  if (!cardStarted_) {
    cardStarted_ = sd_.begin(SdioConfig(FIFO_SDIO));
    if (!cardStarted_) return false;
  }
  bool opened = false;
  for (unsigned index = 0; index < 10000; ++index) {
    snprintf(filename_, sizeof(filename_), "HGT%04u.CSV", index);
    if (!sd_.exists(filename_)) {
      opened = file_.open(filename_, O_WRONLY | O_CREAT | O_EXCL);
      break;
    }
  }
  if (!opened) return false;
  fileOpen_ = true;
  const char header[] =
      "t_us,seq,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,p_pa,temp_c,baro_new\n";
  if (file_.write(header, sizeof(header) - 1) != sizeof(header) - 1 || !file_.sync()) {
    return false;
  }
  head_ = tail_ = count_ = 0;
  sequence_ = dropped_ = 0;
  previousSyncMs_ = millis();
  stopping_ = false;
  recording_ = true;
  return true;
}

void HeightImuSdLogger::fail() {
  failed_ = true;
  recording_ = false;
  startRequested_ = stopping_ = false;
  // No automatic retries after a failed/removed card; fix it and reboot.
}

void HeightImuSdLogger::service(bool safeToAccessCard) {
  if (!safeToAccessCard || failed_) return;
  if (startRequested_) {
    startRequested_ = false;
    if (!openNewFile()) fail();
    return;
  }
  if (!fileOpen_ || file_.isBusy()) return;

  if (count_ != 0) {
    const HeightImuRecord& r = records_[tail_];
    char row[224];
    const int length = snprintf(row, sizeof(row),
        "%lu,%lu,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%.3f,%.2f,%.2f,%u\n",
        (unsigned long)r.timestampUs, (unsigned long)r.sequence,
        r.accelG[0], r.accelG[1], r.accelG[2],
        r.gyroDps[0], r.gyroDps[1], r.gyroDps[2],
        r.rollDeg, r.pitchDeg, r.pressurePa, r.temperatureC, unsigned(r.baroNew));
    if (length <= 0 || length >= int(sizeof(row))
        || file_.write(row, size_t(length)) != size_t(length)) {
      fail();
      return;
    }
    tail_ = (tail_ + 1) % CAPACITY;
    --count_; // Only discard after a successful row write.
  } else if (stopping_) {
    if (!file_.close()) fail();
    fileOpen_ = false;
    stopping_ = false;
  } else if (uint32_t(millis() - previousSyncMs_) >= 1000) {
    if (!file_.sync()) fail();
    previousSyncMs_ = millis();
  }
}

} // namespace itc