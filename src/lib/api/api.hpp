#include <iostream>
#include "httplib.hpp"

using Req = const httplib::Request;
using Res = httplib::Response;

class ApiServer {
private:
    httplib::Server* svr;


    void get_hello(Req &req, Res &res);
public:
    ApiServer();
    ~ApiServer();
};