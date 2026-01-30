#include "gnss_selector.hpp"

void GnssSelector::setGnssData(const sensor_gps_s &gnss_data, int instance)
{
	GnssState &state = _receiver_state[instance];

	state.gnss_data = gnss_data;
	state.updated = true;
}

void GnssSelector::update(hrt_abstime now_us)
{
	_is_new_output_data_available = false;

	const int secondary_instance = _primary_instance == 0 ? 1 : 0;
	updateHealth(_receiver_state[_primary_instance], now_us);
	updateHealth(_receiver_state[secondary_instance], now_us);
	const bool primary_ok   = _receiver_state[_primary_instance].is_healthy;

	if (_selected == -1) {
		// No selection yet
		// Wait for primary to become healthy before making a selection
		if (primary_ok) {
			_selected = _primary_instance;

		} else {
			return;
		}
	}

	if (_selected == _primary_instance) {
		// Currently primary
		const bool secondary_ever_online = _receiver_state[secondary_instance].gnss_data.timestamp != 0;

		if (!primary_ok && secondary_ever_online) {
			// Switch to secondary if primary is unhealthy
			// We specifically don't check for health, since if the primary is an MB Rover,
			// its message rate may be lower than expected until we switch.
			_selected = secondary_instance;
		}

	} else if (_selected == secondary_instance) {
		// Currently secondary
		if (primary_ok) {
			// Primary is healthy, switch back after hysteresis

			// If reconfiguring the secondary instance from rover mode to normal mode, it
			// will be unhealthy for a short period of time. We don't want to switch back to
			// primary until it has been healthy for the hysteresis period, even if the secondary
			// is unhealthy.

			// The risk of recurring primary failures is higher than that of a dual failure
			// when reconfiguring the secondary instance.

			if (now_us - _receiver_state[_primary_instance].last_unhealthy_us >= _hysteresis_us) {
				_selected = _primary_instance;
			}

		}
	}

	if (_receiver_state[_selected].updated) {
		_is_new_output_data_available = true;
		_receiver_state[_selected].updated = false;
	}
}

void GnssSelector::updateHealth(GnssState &gnss, uint64_t now_us)
{
	bool check_failed{false};

	if (gnss.gnss_data.timestamp == 0) {
		check_failed = true;
	}

	// Mirror checks from Ekf::runSimplifiedGnssChecks, but with lower thresholds.

	// We use the same thresholds as in the "regular" GPS EKF checks (which were previously used during
	// flight as well). They are stricter than the simplified ones, so that we don't switch to secondary
	// too late, but don't risk EKF rejecting the secondary either.

	// We have experience flying with these thresholds, so false positives should be rare. Review of in-air
	// antenna disconnections indicate that these thresholds should trigger within 2-3 seconds of antenna disconnection.
	// Speed variance appears to be a particularly good early indicator, trigering within 1 seconds of the first
	// signs of discrepancy between primary and secondary.

	if ((gnss.gnss_data.fix_type < 3) ||
	    (gnss.gnss_data.eph > 5.f) || // = hacc
	    (gnss.gnss_data.epv > 8.f) || // = vacc
	    (gnss.gnss_data.s_variance_m_s > 1.f) // = sacc
	   ) {
		check_failed = true;
	}

	if (now_us - gnss.gnss_data.timestamp >= _msg_timeout_us) {
		check_failed = true;
	}


	if (check_failed) {
		gnss.is_healthy = false;
		gnss.last_unhealthy_us = now_us;
		return;
	}

	gnss.is_healthy = true;
}
