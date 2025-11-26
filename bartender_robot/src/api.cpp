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

    // 서버 시작
    int api_port = std::getenv("API_PORT") ? std::atoi(std::getenv("API_PORT")) : 8080;
    if (api_port <= 0 || api_port > 65535)
    {
        std::cerr << "Invalid API_PORT value. Using default port 8080." << std::endl;
        api_port = 8080;
    }

    std::cout << "API server listening on port " << api_port << std::endl;
    this->svr->listen("localhost", api_port);
}

void ApiServer::get_test(Req &req, Res &res)
{
    json response_json;
    response_json["message"] = "success";

    res.set_content(response_json.dump(), "application/json");
    res.status = 200;
}