#include <memory>

#include <iostream>
#include <ctime>
#include <zconf.h>
#include <google/protobuf/stubs/common.h>
#include "include/Map.h"
#include "include/Macros.h"
#include "include/HttpServer.h"
#include "map.pb.h"

std::unique_ptr<HttpServer> g_httpHandler;

void on_initialize(const std::string& address) {

    uri_builder uri(address);


    auto addr = uri.to_uri().to_string();
    g_httpHandler = std::make_unique<HttpServer>(addr);
    std::cout << "Launching http handler..." << std::endl;
    g_httpHandler->open().wait();

    std::cout << "Listening for requests at: " << addr << std::endl;
}

void on_shutdown()
{
    std::cout << "Closing http handler..." << std::endl;
    g_httpHandler->close().wait();
    std::cout << "Http handler closed" << std::endl;
}

void writeTestMap() {
    auto *posBoid1 = new Protobuf::Pos2D();
    posBoid1->set_x(5);
    posBoid1->set_y(5);

    auto *dirBoid1 = new Protobuf::Pos2D();
    dirBoid1->set_x(1);
    dirBoid1->set_y(0);

    auto *flock = new Protobuf::Flock();

    Protobuf::Boid *boid1 = flock->add_boids();
    boid1->set_allocated_position(posBoid1);
    boid1->set_allocated_direction(dirBoid1);

    auto *posBoid2 = new Protobuf::Pos2D();
    posBoid2->set_x(6);
    posBoid2->set_y(2);

    auto *dirBoid2 = new Protobuf::Pos2D();
    dirBoid2->set_x(2);
    dirBoid2->set_y(1);


    Protobuf::Boid *boid2 = flock->add_boids();
    boid2->set_allocated_position(posBoid2);
    boid2->set_allocated_direction(dirBoid2);


    Protobuf::Input input;

    input.set_allocated_flock(flock);


    auto *proto_map = new Protobuf::Map();

    auto *dimensions = new Protobuf::Pos2D;
    dimensions->set_x(10);
    dimensions->set_y(10);

    proto_map->set_allocated_dimensions(dimensions);

    input.set_allocated_map(proto_map);

    fstream output("input_test.protobinary", ios::out | ios::trunc | ios::binary);
    if (!input.SerializeToOstream(&output)) {
        cerr << "Failed to write map." << endl;
        return;
    }
}


int main() {

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    std::string port = "8080";
    std::string address = "http://127.0.0.1:";
    address.append(port);

    on_initialize(address);
    std::cout << "Press ENTER to exit." << std::endl;

    std::string line;
    std::getline(std::cin, line);

    on_shutdown();

//    writeTestMap();

    google::protobuf::ShutdownProtobufLibrary();

/*

    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    std::cout << "Hello, World!" << std::endl;

    Flock flock;
    int nbOfBoids = 10;
    for (int i = 0; i < nbOfBoids; ++i) {
        Boid boid;
        flock.addBoid(boid);
    }
    Map map(flock, Pos2D(WIDTH, HEIGHT));

    clock_t t;
    while (true) {
        t = clock();
        flock.update((float)t / CLOCKS_PER_SEC);

        usleep(200 * 1000);
        map.display();
    }

    return 0;*/
}





