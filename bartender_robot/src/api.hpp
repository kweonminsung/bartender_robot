#include <iostream>
#include <cstdlib>
#include <functional>
#include <nlohmann/json.hpp>
#include <httplib/httplib.hpp>

using Req = const httplib::Request;
using Res = httplib::Response;
using json = nlohmann::json;

class ApiServer
{
private:
    httplib::Server *svr;
    std::function<void(const std::string&)> play_plan_callback_;
    int api_port_;

    void get_test(Req &req, Res &res);
    void post_play_plan(Req &req, Res &res);

public:
    ApiServer();
    void set_play_plan_callback(std::function<void(const std::string&)> callback);
    void start_listening();
    ~ApiServer()
    {
        delete this->svr;
    }
};