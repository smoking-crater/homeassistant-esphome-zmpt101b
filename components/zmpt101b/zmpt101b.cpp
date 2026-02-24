#include "zmpt101b.h"
#include <cmath>
#include "esphome/core/log.h"

namespace esphome {
namespace zmpt101b {

static const char *const TAG = "zmpt101b";

int ZMPT101BSensor::getZeroPoint_() {
  uint32_t Vsum = 0;
  uint32_t measurements_count = 0;
  uint32_t t_start = micros();

  while (micros() - t_start < this->period_) {
    Vsum += adc_sensor_->sample() * ADC_SCALE;
    measurements_count++;
  }

  return measurements_count ? (Vsum / measurements_count) : 0;
}

void ZMPT101BSensor::loop() {
  // ---- 1) Compute raw RMS estimate (your original logic) ----
  double readingVoltage = 0.0;

  for (uint8_t i = 0; i < this->loop_count_; i++) {
    int zeroPoint = this->getZeroPoint_();

    int32_t Vnow = 0;
    uint32_t Vsum = 0;
    uint32_t measurements_count = 0;

    uint32_t t_start = micros();
    while (micros() - t_start < this->period_) {
      Vnow = (adc_sensor_->sample() * ADC_SCALE) - zeroPoint;
      Vsum += (uint32_t) (Vnow * Vnow);
      measurements_count++;

      // Optional: yield periodically if you see watchdog/lag issues.
      // if ((measurements_count & 0x3F) == 0) {
      //   delay(0);
      // }
    }

    if (measurements_count == 0) {
      ESP_LOGW(TAG, "No ADC measurements captured (period_=%u)", (unsigned) this->period_);
      continue;
    }

    double v_out_rms = sqrt((double) Vsum / (double) measurements_count) / ADC_SCALE * VREF;
    double mains_v_rms = v_out_rms * (1000.0 / this->sensitivity_);
    readingVoltage += mains_v_rms;
  }

  if (this->loop_count_ == 0) {
    ESP_LOGW(TAG, "loop_count_ is 0; cannot compute voltage");
    return;
  }

  const double x = readingVoltage / this->loop_count_;  // raw RMS estimate

  // ---- 2) Best-practice: fixed publish rate + low-pass smoothing ----
  // Publish once per second to prevent jittery high-rate updates.
  static uint32_t last_pub_ms = 0;
  const uint32_t now_ms = millis();
  const uint32_t publish_interval_ms = 1000;  // 1 Hz reporting

  if (now_ms - last_pub_ms < publish_interval_ms) {
    return;
  }
  last_pub_ms = now_ms;

  // Optional: sanity clamp to drop obvious glitches/spikes.
  // Uncomment and adjust range to your locale if desired.
  //
  // if (x < 80.0 || x > 160.0) {
  //   ESP_LOGW(TAG, "Discarding out-of-range voltage sample: %.2f", x);
  //   return;
  // }

  // Exponential Moving Average (EMA) low-pass filter.
  // alpha tunes smoothing: lower = smoother/slower; higher = faster/noisier.
  // With 1 Hz publishing:
  //   alpha ~ 0.10 => ~8-10s smoothing
  //   alpha ~ 0.15 => ~5-7s smoothing (recommended starting point)
  //   alpha ~ 0.20 => ~3-5s smoothing
  static bool filter_init = false;
  static double y = 0.0;
  const double alpha = 0.15;

  if (!filter_init) {
    y = x;
    filter_init = true;
  } else {
    y = y + alpha * (x - y);
  }

  this->publish_state(y);
}

}  // namespace zmpt101b
}  // namespace esphome
