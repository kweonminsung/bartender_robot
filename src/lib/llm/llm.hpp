#include <httplib/httplib.hpp>
#include <nlohmann/json.hpp>
#include <iostream>

using json = nlohmann::json;

class LLMClient
{
private:
    std::string server_host;
    int server_port;
    std::string api_key;
    std::string model_name;
    httplib::SSLClient *client;

public:
    LLMClient();
    ~LLMClient()
    {
        delete this->client;
    }

    std::string request(const std::string &input);
};