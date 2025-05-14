#include "api.hpp"
#include "httplib.hpp"

ApiServer::ApiServer()
{
    this->svr = new httplib::Server();

    this->svr->Get("/test", [this](Req &req, Res &res)
                   { this->get_test(req, res); });

    this->svr->Post("/test/motor/pca9685", [this](Req &req, Res &res)
                    { this->post_test_motor_pca9685(req, res); });

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

void ApiServer::post_test_motor_pca9685(Req &req, Res &res)
{
    json request_json = json::parse(req.body);
    int channel = request_json["channel"];
    int angle = request_json["angle"];

    // PCA9685Servo pca9685Servo;
    // pca9685Servo.rotate(channel, angle);

    json response_json;
    response_json["message"] = "success";
    response_json["channel"] = channel;
    response_json["angle"] = angle;

    res.set_content(response_json.dump(), "application/json");
    res.status = 200;
}