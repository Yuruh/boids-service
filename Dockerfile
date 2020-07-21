FROM ubuntu:20.04
RUN apt-get update

RUN DEBIAN_FRONTEND="noninteractive" apt-get -y install tzdata

RUN apt-get install -y \
    apt-utils \
    cmake \
    g++ \
    zlib1g-dev \
    libcpprest-dev \
    curl \
    libtool \
    automake \
    autoconf \
    unzip


COPY . .


RUN gunzip protobuf-cpp-3.12.3.tar.gz
RUN tar -xvf protobuf-cpp-3.12.3.tar

# Compile protobuf from sources
RUN cd protobuf-3.12.3 && ./configure && make && make install && ldconfig


RUN mkdir build

RUN cd build && cmake .. && make

EXPOSE 8080

CMD ./build/boids

