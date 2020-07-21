#!/usr/bin/env bash

curl -X POST -H "Content-Type:application/octet-stream" --data-binary @input_test.protobinary 127.0.0.1:8080