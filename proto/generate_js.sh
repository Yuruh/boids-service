#!/usr/bin/env bash

protoc --js_out=import_style=commonjs,binary:. *.proto
