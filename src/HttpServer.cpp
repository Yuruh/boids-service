//
// Created by antoine on 17/07/20.
//

#include <cpprest/filestream.h>
#include "../include/HttpServer.h"
#include "../include/Map.h"
#include "../include/Flock.h"
#include "../include/Macros.h"
#include "../include/Parameters.h"
#include "../include/QuadTreeNode.h"

HttpServer::HttpServer() = default;

HttpServer::HttpServer(std::string url): m_listener(url) {
    std::cout << "Listening to " << url << std::endl;
    m_listener.support(methods::GET, std::bind(&HttpServer::handle_get, this, std::placeholders::_1));
    m_listener.support(methods::POST, std::bind(&HttpServer::handle_post, this, std::placeholders::_1));

    m_listener.support(methods::OPTIONS, std::bind(&HttpServer::handle_options, this, std::placeholders::_1));
}

HttpServer::~HttpServer() = default;

void limitInteger(int min, int max, int *value) {
    if (*value > max) {
        *value = max;
    }
    if (*value < min) {
        *value = min;
    }
}

// Todo handle errors and sanitize inputs
void HttpServer::handle_post(http_request message) {


    message.extract_vector().then([=](std::vector<unsigned char> c) {
        std::string s(c.begin(), c.end());
        Protobuf::Input input;

        input.ParseFromString(s);


        Map map;

        map << input.map();


        Flock flock(map);
        flock << input.flock();

        Parameters params;

        params << input.parameters();

        flock.setParams(params);

        std::cout << input.flock().boids().size() << " boids" << std::endl;
        std::cout << input.map().obstacles().size() + 4 << " obstacles" << std::endl;


        int refreshRate = input.imagespersecond();
        limitInteger(1, MAX_FPS, &refreshRate);

        int secondsOfSimulation = input.simulationdurationsec();
        limitInteger(1, MAX_SIM_SECONDS, &secondsOfSimulation);

        float timePerFrame = 1.0f / refreshRate;

        float elapsedSec = 0;
        Protobuf::Output output;

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        /*
         * Simulation every tick
         * This leads to boids deciding about their next movement every tick, so the less FPS the more dumb they are, which doesn't make sense.
         * To change this, we could set a variable "decisionRate", run the update every tick, and set the simulation at the appropriate interval
         * Additionally, how long it takes to compute would not be based on the FPS anymore
         */
        /*
         * Smth that i could also try: Boids choose to "commit" to a certain steer, and for the next X frames they have to steer that way
         * The goal being to make large turn instead of so many small ones
         * Ou plus simplement, ne pas reprendre une décision à chaque frame. Ça ferait une belle opti en plus
         *
         */
        float lastDecision = BOIDS_DECISION_RATE;
        for (int i = 0; i < refreshRate * secondsOfSimulation; ++i) {
            elapsedSec += timePerFrame;

            if (lastDecision >= BOIDS_DECISION_RATE) {
                lastDecision = 0;
                auto boids = flock.restructureQuadtree();

                auto obstaclesVectors = flock.getCloseObstaclesNormalVectors(map);

                flock.update(timePerFrame, map);
            } else {
                flock.updateStayOnCourse(timePerFrame, map);
                lastDecision += timePerFrame;
            }

/*            auto boids = flock.restructureQuadtree();

            auto obstaclesVectors = flock.getCloseObstaclesNormalVectors(map);

            flock.update(timePerFrame, map);*/

            Protobuf::Simulation *simulation = output.add_simulations();
            auto *protoFlock = new Protobuf::Flock();

            flock >> *protoFlock;

            simulation->set_allocated_flock(protoFlock);
            simulation->set_elapsedtimesecond(elapsedSec);

/*
 * todo for interactive debug
 for (int j = 0; j < obstaclesVectors.first.size(); ++j) {
                auto vector = simulation->add_obstaclesnormalvectors();
                vector->set_x(obstaclesVectors.first[j].x);
                vector->set_y(obstaclesVectors.first[j].y);

                auto pos = simulation->add_obstaclesposition();
                pos->set_x(obstaclesVectors.second[j].x);
                pos->set_y(obstaclesVectors.second[j].y);
            }*/
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Simulation generated in " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

        http_response response (status_codes::OK);


        response.headers().add(U("Access-Control-Allow-Origin"), U("*"));
        response.set_body(output.SerializeAsString());
        response.headers().set_content_type("application/octet-stream");
        response.headers().set_cache_control("no-cache");
        message.reply(response);         // reply is done here


    });
}


void HttpServer::handle_get(http_request message) {
    std::cout << message.to_string() << std::endl;
    message.reply(status_codes::OK,message.to_string());
}

void HttpServer::handle_error(pplx::task<void> &t) {
    try
    {
        t.get();
    }
    catch(...)
    {
        // Ignore the error, Log it if a logger is available
    }
}

void HttpServer::handle_options(http_request message) {
    http_response response (status_codes::OK);

    response.headers().add(U("Access-Control-Allow-Origin"), U("*"));
    message.reply(response);         // reply is done here
}

