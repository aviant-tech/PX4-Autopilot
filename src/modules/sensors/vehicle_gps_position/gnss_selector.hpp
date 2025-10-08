#pragma once

#include <cstdint>
#include <cmath>
#include <drivers/drv_hrt.h>

#include <uORB/topics/sensor_gps.h>
#include <px4_platform_common/log.h>

#ifdef UNIT_TEST
#undef PX4_ERR
#define PX4_ERR(fmt, ...) printf(fmt "\n", ##__VA_ARGS__)
#endif

using namespace time_literals;

class GnssSelector
{
public:
	GnssSelector() = default;

	void setGnssData(const sensor_gps_s &gnss_data, int instance);

	void update(hrt_abstime now_us);

	/**
	 * Check if new output data is available.
	 * @return true if the last update resulted in new output data.
	 */
	bool isNewOutputDataAvailable() const { return _is_new_output_data_available; }

	/**
	 * Get the output GNSS data for the currently selected instance.
	 * Should only be called if isNewOutputDataAvailable() returns true
	 * @return a reference to the output GNSS data.
	 */
	const sensor_gps_s &getOutputGnssData() const
	{
		return _receiver_state[_selected == -1 ? 0 : _selected].gnss_data;
	}

	/**
	 * Get the currently selected GNSS instance
	 * @return -1 if no instance is selected, otherwise the selected instance
	*/
	int getSelectedGnss() const { return _selected; }

	// configuration
	void setHysteresis(uint64_t hysteresis_us) { _hysteresis_us = hysteresis_us; }
	void setMsgTimeout(uint64_t timeout_us) { _msg_timeout_us = timeout_us; }
	void setPrimaryInstance(int instance)
	{
		if (instance < 0 || instance >= GNSS_MAX_RECEIVERS) {
			PX4_ERR("Invalid primary instance %d, must be between 0 and %d", instance, GNSS_MAX_RECEIVERS - 1);
			return;
		}

		_primary_instance = instance;
	}

	static constexpr int GNSS_MAX_RECEIVERS = 2;
private:
	struct GnssState {
		sensor_gps_s gnss_data{};
		bool updated{false};

		// health/selection bookkeeping
		bool is_healthy{false};
		uint64_t last_unhealthy_us{0};
	};

	void updateHealth(GnssState &gnss, hrt_abstime now_us);

	// configuration
	uint64_t _hysteresis_us{5_s};
	uint64_t _msg_timeout_us{1_s};
	int _primary_instance{0};

	// state
	GnssState _receiver_state[2] {};
	int _selected{-1};
	bool _is_new_output_data_available{false};
};
