#include <iostream>
#include <cstdlib>
#include "api.hpp"
#include "httplib.hpp"

ApiServer::ApiServer() {
    this->svr = new httplib::Server();

    this->svr->Get("/", [this](Req &req, Res &res) {
        this->get_hello(req, res);
    });

    
    int api_port = std::getenv("API_PORT") ? std::atoi(std::getenv("API_PORT")) : 8080;
    if (api_port <= 0 || api_port > 65535) {
        std::cerr << "Invalid API_PORT value. Using default port 8080." << std::endl;
        api_port = 8080;
    }

    std::cout << "API server listening on port " << api_port << std::endl;
    this->svr->listen("localhost", api_port);
}

ApiServer::~ApiServer() {
    delete this->svr;
}


void ApiServer::get_hello(Req &req, Res &res) {
    res.set_content("Hello, World!", "text/plain");
}