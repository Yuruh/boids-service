//
// Created by antoine on 17/07/20.
//

#ifndef BOIDS_HTTPSERVER_H
#define BOIDS_HTTPSERVER_H


#include <cpprest/details/basic_types.h>
#include <pplx/pplxtasks.h>
#include <cpprest/base_uri.h>
#include <cpprest/http_headers.h>
#include <cpprest/http_listener.h>

using namespace std;
using namespace web;
using namespace http;
using namespace http::experimental::listener;

class HttpServer {
public:
    HttpServer();
    HttpServer(std::string url);
    virtual ~HttpServer();

    pplx::task<void>open(){return m_listener.open();}
    pplx::task<void>close(){return m_listener.close();}

protected:

private:
    void handle_get(http_request message);
    void handle_post(http_request message);
    void handle_error(pplx::task<void>& t);
    http_listener m_listener;
};


#endif //BOIDS_HTTPSERVER_H
