#!/usr/bin/env bash

protoc --cpp_out=.. *.proto

mv ../map.pb.cc ../src
mv ../map.pb.h ../include