FROM px4io/px4-dev-simulation-focal
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Oslo

RUN groupadd -r px4 -g 1000 && useradd -m -u 1000 -r -g px4 px4
USER px4

RUN echo "set auto-load safe-path /" > /home/px4/.gdbinit
RUN echo "handle SIG32 noprint nostop" >> /home/px4/.gdbinit

ENTRYPOINT ["/bin/bash", "-c"]
CMD ["/bin/bash"]
