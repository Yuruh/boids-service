#!/usr/bin/env bash

#outputs both html and md reports

docker run --rm -v $(pwd)/doc:/out -v $(pwd):/protos pseudomuto/protoc-gen-doc
docker run --rm -v $(pwd)/doc:/out -v $(pwd):/protos pseudomuto/protoc-gen-doc --doc_opt=markdown,docs.md
