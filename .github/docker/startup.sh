#!/bin/bash

if [ ! -z "${LOCAL_REPO}" ]; then
  pushd /src/px4_local
  make px4_sitl
  DONT_RUN=1 make px4_sitl gazebo_standard_vtol
fi
  

if [ ! -z "${MULTI_VEHICLE}" ]; then
  # Use last byte of IP address as mavlink system ID
  export SYSTEM_ID=$(ip addr show eth0 | grep inet | sed 's,.*10[.]8[.]1[.]\([0-9]\+\)/.*,\1,')
  sed -i "s,param set MAV_SYS_ID .*,param set MAV_SYS_ID $SYSTEM_ID," ROMFS/px4fmu_common/init.d-posix/rcS
  # Move each vehicle a bit, so they do not stack
  export PX4_HOME_LON=$(echo "$PX4_HOME_LON+(0.001*$SYSTEM_ID)" | bc)
  mavlink-routerd -c /etc/mavlink-router/multi.conf &
else
  mavlink-routerd &
fi

# Start distance sensor mock
python3 /src/distance-sensor/distance_sensor_sim.py &

make px4_sitl gazebo_standard_vtol
