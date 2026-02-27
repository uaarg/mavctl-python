FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y python3 python3-pip netcat-traditional
RUN apt install -y git


WORKDIR /work
RUN git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
WORKDIR /work/ardupilot

COPY patches /work/patches
RUN git apply /work/patches/01-SITL-ubuntu-deps.patch
RUN git apply /work/patches/02-SITL-no-usermod.patch

RUN Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN echo '. ~/venv-ardupilot/bin/activate' >> /root/.profile

RUN . ~/venv-ardupilot/bin/activate && \
    ./waf configure --board sitl && \
    ./waf copter

CMD . /root/venv-ardupilot/bin/activate
#    Tools/autotest/sim_vehicle.py -v Copter --no-mavproxy -w -S 10 --udp
