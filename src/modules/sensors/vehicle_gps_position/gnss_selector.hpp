#pragma once

#include <cstdint>
#include <cmath>
#include <drivers/drv_hrt.h>

#include <uORB/topics/sensor_gps.h>
#include <px4_platform_common/log.h>

#ifdef UNIT_TEST
#undef PX4_ERR
#define PX4_ERR(fmt, ...) printf(fmt "\n", ##__VA_ARGS__)
#undef PX4_WARN
#define PX4_WARN(fmt, ...) printf(fmt "\n", ##__VA_ARGS__)
#undef PX4_INFO
#define PX4_INFO(fmt, ...) printf(fmt "\n", ##__VA_ARGS__)
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

	/**
	 * Why a receiver was considered unhealthy by the last updateHealth() call.
	 * Bitmask, logged on every failover so the trigger is unambiguous in the flight log.
	 */
	enum HealthFlags : uint8_t {
		HEALTH_NO_DATA		= 1 << 0,	///< nothing ever received from this receiver
		HEALTH_FIX_TYPE		= 1 << 1,	///< fix_type < 3
		HEALTH_EPH		= 1 << 2,	///< horizontal accuracy worse than the threshold
		HEALTH_EPV		= 1 << 3,	///< vertical accuracy worse than the threshold
		HEALTH_SPEED_ACC	= 1 << 4,	///< speed accuracy worse than the threshold
		HEALTH_MSG_TIMEOUT	= 1 << 5,	///< no message within SENS_GPS_TOUT
	};

	static constexpr int GNSS_MAX_RECEIVERS = 2;
private:
	struct GnssState {
		sensor_gps_s gnss_data{};
		bool updated{false};

		// health/selection bookkeeping
		bool is_healthy{false};
		uint64_t last_unhealthy_us{0};
		uint8_t unhealthy_reason{0};	///< HealthFlags bitmask from the last updateHealth() call
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
