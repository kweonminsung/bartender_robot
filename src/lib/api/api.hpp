#include <iostream>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include "httplib.hpp"

using Req = const httplib::Request;
using Res = httplib::Response;
using json = nlohmann::json;

class ApiServer
{
private:
    httplib::Server *svr;

    void get_test(Req &req, Res &res);

public:
    ApiServer();
    ~ApiServer();
};