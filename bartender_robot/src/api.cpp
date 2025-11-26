#include "api.hpp"

ApiServer::ApiServer()
{
    this->svr = new httplib::Server();

    // 미들웨어: 모든 요청을 JSON으로 출력
    this->svr->set_pre_routing_handler([](const httplib::Request &req, httplib::Response & /*res*/)
                                       {
        json log_json;
        log_json["method"] = req.method;
        log_json["path"] = req.path;
        log_json["headers"] = req.headers;
        log_json["body"] = req.body;
        std::cout << "[API Request] " << log_json.dump() << std::endl;
        return httplib::Server::HandlerResponse::Unhandled; });

    // 미들웨어: 모든 응답을 JSON으로 출력
    this->svr->set_post_routing_handler([](const httplib::Request & /*req*/, httplib::Response &res)
                                        {
        json log_json;
        log_json["status"] = res.status;
        log_json["headers"] = res.headers;
        log_json["body"] = res.body;
        std::cout << "[API Response] " << log_json.dump() << std::endl; });

    // API 엔드포인트 설정
    this->svr->Get("/test", [this](Req &req, Res &res)
                   { this->get_test(req, res); });
    
    this->svr->Post("/play_plan", [this](Req &req, Res &res)
                    { this->post_play_plan(req, res); });

    // 포트 설정
    api_port_ = std::getenv("API_PORT") ? std::atoi(std::getenv("API_PORT")) : 8080;
    if (api_port_ <= 0 || api_port_ > 65535)
    {
        std::cerr << "Invalid API_PORT value. Using default port 8080." << std::endl;
        api_port_ = 8080;
    }
}

void ApiServer::start_listening()
{
    std::cout << "API server listening on port " << api_port_ << std::endl;
    this->svr->listen("localhost", api_port_);
}

void ApiServer::set_play_plan_callback(std::function<void(const std::string&)> callback)
{
    play_plan_callback_ = callback;
}

void ApiServer::get_test(Req &req, Res &res)
{
    json response_json;
    response_json["message"] = "success";

    res.set_content(response_json.dump(), "application/json");
    res.status = 200;
}

void ApiServer::post_play_plan(Req &req, Res &res)
{
    json response_json;
    
    try {
        json request_json = json::parse(req.body);
        std::string csv_path = request_json.value("csv_path", "/home/kevin/bartender_robot/captured_plan_20251127_001513.csv");
        
        if (play_plan_callback_) {
            play_plan_callback_(csv_path);
            response_json["status"] = "success";
            response_json["message"] = "Plan execution started";
            response_json["csv_path"] = csv_path;
            res.status = 200;
        } else {
            response_json["status"] = "error";
            response_json["message"] = "Plan player callback not set";
            res.status = 500;
        }
    } catch (const std::exception& e) {
        response_json["status"] = "error";
        response_json["message"] = e.what();
        res.status = 400;
    }
    
    res.set_content(response_json.dump(), "application/json");
}