
FROM ubuntu:22.04

RUN apt-get -y update
RUN apt-get -y install git
RUN apt-get -y install cmake
RUN apt install -y gcc clang clang-tools cmake python3
RUN apt-get -y install sudo

ENV PKG_PATH=/root/robotics_playground/intro-to-robotics
COPY --chown=root include ${PKG_PATH}/include
COPY --chown=root src ${PKG_PATH}/src
COPY --chown=root test ${PKG_PATH}/test
COPY --chown=root CMakeLists.txt ${PKG_PATH}/CMakeLists.txt

RUN cd ${PKG_PATH} && \
    git clone https://gitlab.com/libeigen/eigen.git && \
    git clone https://github.com/google/googletest.git && \
    mkdir build && cd build && \
    cmake ..

RUN cd ${PKG_PATH}/build && \
    make

COPY --chown=root /scripts/docker/run_unit_test.sh /usr/bin/run_unit_test
RUN cd usr/bin && \
    sudo chmod +x run_unit_test
CMD ["run_unit_test"]
