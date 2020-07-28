# boids-service
C++ Microservice exposing HTTP endpoint to compute flocking simulation

![Simulation gif](showcase/boids.gif)

## Features

### Done

* Custom protocol to build simulation using protobuf.
* Boids rules implementation: cohesion, separation and alignment
* Custom rule: obstacle evasion

## To come

* Quadtree optimisation

# Protocol Documentation
<a name="top"></a>

## Table of Contents

- [map.proto](#map.proto)
    - [Boid](#Protobuf.Boid)
    - [Flock](#Protobuf.Flock)
    - [Input](#Protobuf.Input)
    - [Line](#Protobuf.Line)
    - [Map](#Protobuf.Map)
    - [Output](#Protobuf.Output)
    - [Pos2D](#Protobuf.Pos2D)
    - [Simulation](#Protobuf.Simulation)
  
- [Scalar Value Types](#scalar-value-types)



<a name="map.proto"></a>
<p align="right"><a href="#top">Top</a></p>

## map.proto



<a name="Protobuf.Boid"></a>

### Boid



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| position | [Pos2D](#Protobuf.Pos2D) |  |  |
| direction | [Pos2D](#Protobuf.Pos2D) |  |  |






<a name="Protobuf.Flock"></a>

### Flock



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| boids | [Boid](#Protobuf.Boid) | repeated | max 1000 |






<a name="Protobuf.Input"></a>

### Input



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| map | [Map](#Protobuf.Map) |  |  |
| flock | [Flock](#Protobuf.Flock) |  |  |
| simulationDurationSec | [int32](#int32) |  | max 10 |
| imagesPerSecond | [int32](#int32) |  | max 60 |






<a name="Protobuf.Line"></a>

### Line



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| a | [Pos2D](#Protobuf.Pos2D) |  |  |
| b | [Pos2D](#Protobuf.Pos2D) |  |  |






<a name="Protobuf.Map"></a>

### Map



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| dimensions | [Pos2D](#Protobuf.Pos2D) |  |  |
| obstacles | [Line](#Protobuf.Line) | repeated |  |






<a name="Protobuf.Output"></a>

### Output



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| simulations | [Simulation](#Protobuf.Simulation) | repeated |  |






<a name="Protobuf.Pos2D"></a>

### Pos2D



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| x | [float](#float) |  |  |
| y | [float](#float) |  |  |






<a name="Protobuf.Simulation"></a>

### Simulation



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| flock | [Flock](#Protobuf.Flock) |  |  |
| elapsedTimeSecond | [float](#float) |  |  |