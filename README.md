[![license](http://img.shields.io/badge/license-MIT-red.svg?style=flat)](http://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/github/v/tag/yuruh/boids-service)](https://github.com/Yuruh/boids-service/releases)
[![size](https://img.shields.io/docker/image-size/yuruh/boids?color=blueviolet)](https://hub.docker.com/r/yuruh/boids)

# boids-service
C++ Microservice exposing HTTP endpoint to compute flocking simulation

![Simulation gif](showcase/boids.gif)

## Demo

[Available here](https://boids-demo.yuruh.fr)

## Features

### Done

* HTTP Endpoint
* Custom protocol to build simulation using protobuf.
* Boids rules implementation: cohesion, separation and alignment
* Custom rule: obstacle evasion

## To come

* Quadtree optimisation
* Better field of view (boids shouldn't see right behind)
* Better obstacle evasion
* Better steering physic (avoid clipping)
* Additional herd behaviour (predation, feeding, ...) 