#include <gtest/gtest.h>
#include <drivers/drv_hrt.h>
#include "gnss_selector.hpp"

using namespace time_literals;

class GnssSelectorTest : public ::testing::Test
{
public:
	sensor_gps_s getDefaultGpsData();

	// Feed new GNSS data for given instance at current time
	void push(GnssSelector &sel, sensor_gps_s &gps, int instance)
	{
		gps.timestamp = _time_now_us;
		sel.setGnssData(gps, instance);
		sel.update(_time_now_us);
	}

	// Advance simulated time
	void advance(uint64_t dt) { _time_now_us += dt; }

	uint64_t _time_now_us{1000000};
};

sensor_gps_s GnssSelectorTest::getDefaultGpsData()
{
	sensor_gps_s gps_data{};
	gps_data.timestamp = _time_now_us - 10e3;
	gps_data.time_utc_usec = 0;
	gps_data.lat = 47e7;
	gps_data.lon = 9e7;
	gps_data.alt = 800e3;
	gps_data.alt_ellipsoid = 800e3;
	gps_data.s_variance_m_s = 0.2f;
	gps_data.c_variance_rad = 0.5f;
	gps_data.eph = 0.7f;
	gps_data.epv = 1.2f;
	gps_data.hdop = 1.f;
	gps_data.vdop = 1.f;
	gps_data.noise_per_ms = 20;
	gps_data.jamming_indicator = 40;
	gps_data.vel_m_s = 1.f;
	gps_data.vel_n_m_s = 1.f;
	gps_data.vel_e_m_s = 1.f;
	gps_data.vel_d_m_s = 1.f;
	gps_data.cog_rad = 0.f;
	gps_data.timestamp_time_relative = 0;
	gps_data.heading = NAN;
	gps_data.heading_offset = 0.f;
	gps_data.fix_type = 4;
	gps_data.vel_ned_valid = true;
	gps_data.satellites_used = 8;

	return gps_data;
}


TEST_F(GnssSelectorTest, noData)
{
	GnssSelector sel;

	EXPECT_EQ(sel.getSelectedGnss(), -1);
	EXPECT_FALSE(sel.isNewOutputDataAvailable());

	// WHEN: update called without any data
	sel.update(_time_now_us);

	// THEN: still no selection
	EXPECT_EQ(sel.getSelectedGnss(), -1);
	EXPECT_FALSE(sel.isNewOutputDataAvailable());
}

TEST_F(GnssSelectorTest, singleReceiver)
{
	GnssSelector sel;
	sel.setPrimaryInstance(0);
	sel.setMsgTimeout(1_s);

	sensor_gps_s gps = getDefaultGpsData();

	// WHEN: first sample arrives
	push(sel, gps, 0);

	// WHEN: another fresh sample arrives later
	advance(200_ms);
	push(sel, gps, 0);

	// THEN: primary selected, new output available
	EXPECT_EQ(sel.getSelectedGnss(), 0);
	EXPECT_TRUE(sel.isNewOutputDataAvailable());

	// BUT WHEN: update called without new data
	sel.update(_time_now_us);

	// THEN: no new output
	EXPECT_FALSE(sel.isNewOutputDataAvailable());
}

TEST_F(GnssSelectorTest, fallbackNoFix)
{
	GnssSelector sel;
	sel.setPrimaryInstance(0);

	sensor_gps_s gps0 = getDefaultGpsData();
	sensor_gps_s gps1 = getDefaultGpsData();

	// GIVEN: both receivers send valid data
	push(sel, gps0, 0);
	push(sel, gps1, 1);
	EXPECT_EQ(sel.getSelectedGnss(), 0);

	// WHEN: primary loses fix
	advance(100_ms);
	gps0.fix_type = 0;
	push(sel, gps0, 0);

	// THEN: switch to secondary
	EXPECT_EQ(sel.getSelectedGnss(), 1);
}

TEST_F(GnssSelectorTest, fallbackTimeout)
{
	GnssSelector sel;
	sel.setPrimaryInstance(0);
	sel.setMsgTimeout(1_s);

	sensor_gps_s gps0 = getDefaultGpsData();
	sensor_gps_s gps1 = getDefaultGpsData();

	// GIVEN: both receivers updated
	push(sel, gps0, 0);
	push(sel, gps1, 1);
	EXPECT_EQ(sel.getSelectedGnss(), 0);

	// WHEN: no updated from primary until just before timeout
	advance(1_s - 1_us);
	push(sel, gps1, 1);

	// THEN: still primary
	EXPECT_EQ(sel.getSelectedGnss(), 0);

	// BUT WHEN: timeout reached
	advance(1_us);
	sel.update(_time_now_us);

	// THEN: switched to secondary
	EXPECT_EQ(sel.getSelectedGnss(), 1);
}

TEST_F(GnssSelectorTest, hysteresisBeforeSwitchBackToPrimary)
{
	GnssSelector sel;
	sel.setPrimaryInstance(0);
	sel.setMsgTimeout(1_s);
	sel.setHysteresis(1_s);

	sensor_gps_s gps0 = getDefaultGpsData();
	sensor_gps_s gps1 = getDefaultGpsData();

	// GIVEN: primary selected
	push(sel, gps0, 0);
	push(sel, gps1, 1);
	EXPECT_EQ(sel.getSelectedGnss(), 0);

	// WHEN: primary goes unhealthy
	advance(100_ms);
	gps0.fix_type = 0;
	push(sel, gps0, 0);
	push(sel, gps1, 1);
	EXPECT_EQ(sel.getSelectedGnss(), 1);

	// WHEN: primary recovers but hysteresis not passed
	advance(1_s - 1_us);
	gps0.fix_type = 4;
	push(sel, gps0, 0);
	push(sel, gps1, 1);
	EXPECT_EQ(sel.getSelectedGnss(), 1);

	// BUT WHEN: hysteresis time elapsed
	advance(1_us);
	push(sel, gps0, 0);
	push(sel, gps1, 1);

	// THEN: switch back to primary
	EXPECT_EQ(sel.getSelectedGnss(), 0);
}

TEST_F(GnssSelectorTest, initializeWithOnlySecondary)
{
	GnssSelector sel;
	sel.setPrimaryInstance(0);

	sensor_gps_s gps0 = getDefaultGpsData();
	sensor_gps_s gps1 = getDefaultGpsData();
	gps0.fix_type = 0;

	// WHEN: only secondary provides good data
	advance(200_ms);
	push(sel, gps0, 0);
	push(sel, gps1, 1);

	// THEN: no selection until primary is valid
	EXPECT_EQ(sel.getSelectedGnss(), -1);
	EXPECT_FALSE(sel.isNewOutputDataAvailable());
}

TEST_F(GnssSelectorTest, primaryRecoversImmediatelyWhenSecondaryUnhealthy)
{
	GnssSelector sel;
	sel.setPrimaryInstance(0);
	sel.setHysteresis(500_ms);

	sensor_gps_s gps0 = getDefaultGpsData();
	sensor_gps_s gps1 = getDefaultGpsData();

	// GIVEN: primary selected
	push(sel, gps0, 0);
	push(sel, gps1, 1);
	EXPECT_EQ(sel.getSelectedGnss(), 0);

	// WHEN: primary unhealthy
	advance(200_ms);
	gps0.fix_type = 0;
	push(sel, gps0, 0);
	push(sel, gps1, 1);
	// THEN: switched to secondary
	EXPECT_EQ(sel.getSelectedGnss(), 1);

	// WHEN: primary recovers, secondary unhealthy
	advance(200_ms);
	gps0.fix_type = 4;
	gps1.fix_type = 0;
	push(sel, gps0, 0);
	push(sel, gps1, 1);

	// THEN: switch back to primary immediately
	EXPECT_EQ(sel.getSelectedGnss(), 0);
}

TEST_F(GnssSelectorTest, noSwitchIfPrimaryAlwaysHealthy)
{
	GnssSelector sel;
	sel.setPrimaryInstance(0);

	sensor_gps_s gps0 = getDefaultGpsData();
	sensor_gps_s gps1 = getDefaultGpsData();

	// GIVEN: healthy updates
	push(sel, gps0, 0);
	push(sel, gps1, 1);

	// THEN: always primary
	EXPECT_EQ(sel.getSelectedGnss(), 0);
}

TEST_F(GnssSelectorTest, primaryRecoversAfterTimeout)
{
	GnssSelector sel;
	sel.setPrimaryInstance(0);
	sel.setMsgTimeout(500_ms);
	sel.setHysteresis(0_ms);

	sensor_gps_s gps0 = getDefaultGpsData();
	sensor_gps_s gps1 = getDefaultGpsData();

	// GIVEN: primary selected
	push(sel, gps0, 0);
	push(sel, gps1, 1);
	EXPECT_EQ(sel.getSelectedGnss(), 0);

	// WHEN: no updates from primary
	advance(600_ms);
	push(sel, gps1, 1);
	// THEN: switched to secondary
	EXPECT_EQ(sel.getSelectedGnss(), 1);

	// WHEN: primary sends fresh data
	advance(200_ms);
	push(sel, gps0, 0);
	push(sel, gps1, 1);

	// THEN: after hysteresis, switch back
	advance(300_ms);
	push(sel, gps0, 0);
	push(sel, gps1, 1);
	EXPECT_EQ(sel.getSelectedGnss(), 0);
}

TEST_F(GnssSelectorTest, primaryNotZero)
{
	GnssSelector sel;
	sel.setPrimaryInstance(1);
	sel.setMsgTimeout(1_s);

	sensor_gps_s gps0 = getDefaultGpsData();
	sensor_gps_s gps1 = getDefaultGpsData();

	// GIVEN: both healthy
	push(sel, gps0, 0);
	push(sel, gps1, 1);
	EXPECT_EQ(sel.getSelectedGnss(), 1);

	// WHEN: primary (1) unhealthy
	advance(200_ms);
	gps1.fix_type = 0;
	push(sel, gps0, 0);
	push(sel, gps1, 1);

	// THEN: switch to secondary (0)
	EXPECT_EQ(sel.getSelectedGnss(), 0);
}
