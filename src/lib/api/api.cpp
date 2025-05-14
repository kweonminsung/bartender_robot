#include "api.hpp"
#include "httplib.hpp"
#include <cstdlib>

ApiServer::ApiServer()
{
    this->svr = new httplib::Server();

    this->svr->Get("/test", [this](Req &req, Res &res)
                   { this->get_test(req, res); });

    int api_port = std::getenv("API_PORT") ? std::atoi(std::getenv("API_PORT")) : 8080;
    if (api_port <= 0 || api_port > 65535)
    {
        std::cerr << "Invalid API_PORT value. Using default port 8080." << std::endl;
        api_port = 8080;
    }

    std::cout << "API server listening on port " << api_port << std::endl;
    this->svr->listen("localhost", api_port);
}

ApiServer::~ApiServer()
{
    delete this->svr;
}

void ApiServer::get_test(Req &req, Res &res)
{
    json response_json;
    response_json["message"] = "success";

    res.set_content(response_json.dump(), "application/json");
    res.status = 200;
}