//
// Created by antoine on 17/07/20.
//

#include <cpprest/filestream.h>
#include "../include/HttpServer.h"
#include "../include/Map.h"
#include "../include/Flock.h"

HttpServer::HttpServer() = default;

HttpServer::HttpServer(std::string url): m_listener(url) {
    m_listener.support(methods::GET, std::bind(&HttpServer::handle_get, this, std::placeholders::_1));
    m_listener.support(methods::POST, std::bind(&HttpServer::handle_post, this, std::placeholders::_1));
}

HttpServer::~HttpServer() = default;

// Todo handle errors and sanitize inputs
void HttpServer::handle_post(http_request message) {

//    std::cout << message.body().extract() << std::endl;

    message.extract_vector().then([=](std::vector<unsigned char> c) {

        std::string s(c.begin(), c.end());
        Protobuf::Input input;

        input.ParseFromString(s);
//        std::cout <<  input.map().dimensions().x() << std::endl;

        Flock flock;
        flock << input.flock();

        Map map;

        map << input.map();

//        map.display();

        // 600 frames generated
        int refreshRate = 60;
        int secondsOfSimulation = 30;
        float timePerFrame = 1.0f / refreshRate;

        float elapsedSec = 0;
        Protobuf::Output output;
        for (int i = 0; i < refreshRate * secondsOfSimulation; ++i) {
            elapsedSec += timePerFrame;
            flock.update(timePerFrame, map);

            Protobuf::Simulation *simulation = output.add_simulations();
            auto *protoFlock = new Protobuf::Flock();
            flock >> *protoFlock;
            simulation->set_allocated_flock(protoFlock);
            simulation->set_elapsedtimesecond(elapsedSec);

        }

        http_response response (status_codes::OK);


        response.headers().add(U("Access-Control-Allow-Origin"), U("*"));
        response.set_body(output.SerializeAsString());
        response.headers().set_content_type("application/octet-stream");
        message.reply(response);         // reply is done here


    });
}


void HttpServer::handle_get(http_request message) {
    std::cout << message.to_string() << std::endl;
    message.reply(status_codes::OK,message.to_string());

//    message.body();

/*    auto paths = http::uri::split_path(http::uri::decode(message.relative_uri().path()));

    message.relative_uri().path();

    concurrency::streams::fstream::open_istream(U("static/index.html"), std::ios::in).then([=](concurrency::streams::istream is)
                                                                                           {
                                                                                               message.reply(status_codes::OK, is,  U("text/html"))
                                                                                                       .then([](pplx::task<void> t)
                                                                                                             {
                                                                                                                 try{
                                                                                                                     t.get();
                                                                                                                 }
                                                                                                                 catch(...){
                                                                                                                     //
                                                                                                                 }
                                                                                                             });
                                                                                           }).then([=](pplx::task<void>t)
                                                                                                                                                                                        {
                                                                                                                                                                                            try{
                                                                                                                                                                                                t.get();
                                                                                                                                                                                            }
                                                                                                                                                                                            catch(...){
                                                                                                                                                                                                message.reply(status_codes::InternalError,U("INTERNAL ERROR "));
                                                                                                                                                                                            }
                                                                                                                                                                                        });
*/
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

