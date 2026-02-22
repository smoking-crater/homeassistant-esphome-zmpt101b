#include "zmpt101b.h"
#include <cmath>
#include "esphome/core/log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

inline void cpu_yield_every(uint32_t &counter, uint32_t every) {
  if ((++counter % every) == 0) {
    // 0 ticks yields without a real delay; 1 tick is a tiny sleep (~1ms typical)
    vTaskDelay(1);
  }
}

namespace esphome {
namespace zmpt101b {

static const char *const TAG = "zmpt101b";

int ZMPT101BSensor::getZeroPoint_() {
	uint32_t Vsum = 0;
	uint32_t measurements_count = 0;
	uint32_t t_start = micros();
	uint32_t yield_counter = 0;

	while (micros() - t_start < this->period_) {
		Vsum += adc_sensor_->sample() * ADC_SCALE;
		measurements_count++;
		cpu_yield_every(yield_counter, 64);  // tune 32/64/128
	}

	return measurements_count ? (Vsum / measurements_count) : 0;
}

void ZMPT101BSensor::loop() {
	double readingVoltage = 0.0;

	for (uint8_t i = 0; i < this->loop_count_; i++) {
		int zeroPoint = this->getZeroPoint_();

		int32_t Vnow = 0;
		uint32_t Vsum = 0;
		uint32_t measurements_count = 0;

		uint32_t t_start = micros();
		uint32_t yield_counter = 0;
		
		while (micros() - t_start < this->period_) {
			Vnow = (adc_sensor_->sample() * ADC_SCALE) - zeroPoint;
			Vsum += Vnow * Vnow;
			measurements_count++;
			cpu_yield_every(yield_counter, 64);  // tune
		}

		double v_out_rms = sqrt((double)Vsum / (double)measurements_count) / ADC_SCALE * VREF;
		double mains_v_rms = v_out_rms * (1000.0 / this->sensitivity_);
		readingVoltage += mains_v_rms;
	}

	this->publish_state(readingVoltage / this->loop_count_);
}

}  // namespace zmpt101b
}  // namespace esphome
