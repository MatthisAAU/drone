#pragma once

#include <SdFat.h>
#include "HeightImuLogFormat.h"

namespace itc {

// CSV logger for the standalone IMU + barometer height test.
// Same structure as HeightSdLogger: RAM queue, card access only in service().
// It may block on SD: do not put it into flight control without a timing design.
class HeightImuSdLogger {
 public:
  void begin();
  void requestStart();
  void requestStop();
  void sample(const HeightImuRecord& reading);
  void service(bool safeToAccessCard);

  bool recording() const { return recording_; }
  bool failed() const { return failed_; }
  bool closed() const { return !fileOpen_ && !startRequested_ && !failed_; }
  unsigned queuedSamples() const { return count_; }
  uint32_t droppedSamples() const { return dropped_; }
  const char* filename() const { return filename_; }

 private:
  static constexpr unsigned CAPACITY = 256;   // ~1.3 s buffer at 200 Hz
  HeightImuRecord records_[CAPACITY];
  unsigned head_ = 0;
  unsigned tail_ = 0;
  unsigned count_ = 0;
  uint32_t sequence_ = 0;
  uint32_t dropped_ = 0;
  bool recording_ = false;
  bool startRequested_ = false;
  bool stopping_ = false;
  bool fileOpen_ = false;
  bool failed_ = false;
  bool cardStarted_ = false;
  uint32_t previousSyncMs_ = 0;
  char filename_[13] = {};
  SdFs sd_;
  FsFile file_;

  bool openNewFile();
  void fail();
};

} // namespace itc