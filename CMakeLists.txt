cmake_minimum_required(VERSION 3.10)
project(boids)

set(Casablanca_LIBRARIES "-lboost_system -lcrypto -lssl -lcpprest -lpthread")

set(PROTOBUF_LIBRARY "-lprotobuf")

set(CMAKE_CXX_STANDARD 17)

#ADD_SUBDIRECTORY(proto)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

add_executable(boids main.cpp map.pb.h map.pb.cc src/Map.cpp include/Map.h src/Pos2D.cpp include/Pos2D.h src/Core.cpp include/Core.h src/Boid.cpp include/Boid.h src/Flock.cpp include/Flock.h include/Macros.h src/HttpServer.cpp include/HttpServer.h Line.cpp Line.h)

TARGET_LINK_LIBRARIES(boids ${Casablanca_LIBRARIES} ${PROTOBUF_LIBRARY})
#TARGET_LINK_LIBRARIES(boids proto ${PROTOBUF_LIBRARY})