#!/usr/bin/env bash

docker build -t yuruh/boids:$(git describe --abbrev=0) .

docker push yuruh/boids:$(git describe --abbrev=0)

# docker run -a stdin -a stdout -p "8080:8080" -it yuruh/boids:$(git describe --abbrev=0)
