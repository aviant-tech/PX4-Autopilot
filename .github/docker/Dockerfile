FROM px4io/px4-dev-simulation-focal
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Oslo
RUN apt-get update && apt-get install -y \
  git \
  iproute2
RUN pip3 install meson
RUN groupadd -r px4 -g 1000 && useradd -m -u 1000 -r -g px4 px4

WORKDIR /src
RUN chown px4:px4 .
RUN git clone https://github.com/mavlink-router/mavlink-router.git
WORKDIR /src/mavlink-router
ARG MAVLINK_ROUTER_VERSION
RUN git checkout ${MAVLINK_ROUTER_VERSION} && git submodule update --init --recursive
RUN meson setup build . && ninja -C build install
COPY .github/docker/main.conf /etc/mavlink-router/
COPY .github/docker/multi.conf /etc/mavlink-router/
WORKDIR /src
RUN rm -rf mavlink-router

WORKDIR /src/px4
RUN chown px4:px4 .
USER px4

WORKDIR /src
COPY --chown=px4:px4 . px4
WORKDIR /src/px4
ENV DONT_RUN=1

RUN make px4_sitl
RUN make px4_sitl gazebo_standard_vtol
ENV DONT_RUN=

# Copy distance sensor mock script and install dependencies
USER root
WORKDIR /install
RUN chown px4:px4 .
RUN apt-get update && apt-get install -y \
  python3 \
  python3-pip
USER px4
COPY .github/docker/requirements.txt /install/requirements.txt
RUN pip3 install -r /install/requirements.txt
WORKDIR /src/distance-sensor
RUN chown px4:px4 .
COPY --chown=px4:px4 .github/docker/distance_sensor_sim.py ./

USER px4
COPY .github/docker/startup.sh /src/px4
WORKDIR /src/px4
CMD ["./startup.sh"]
