#include "llm.hpp"

LLMClient::LLMClient()
{
    const char *server_host = std::getenv("LLM_SERVER_HOST");
    const char *model_name = std::getenv("LLM_MODEL_NAME");
    const char *api_key = std::getenv("LLM_API_KEY");

    if (!server_host || !model_name || !api_key)
    {
        throw std::runtime_error("LLM environment variable not set");
    }

    this->server_host = server_host;
    this->model_name = model_name;
    this->api_key = api_key;

    this->client = new httplib::SSLClient(server_host);
    this->client->set_bearer_token_auth(api_key);

    // Disable cert verification
    this->client->enable_server_certificate_verification(false);

    // Disable host verification
    this->client->enable_server_hostname_verification(false);
}

std::string LLMClient::request(const std::string &input)
{
    json request_json;
    request_json["model"] = this->model_name;
    request_json["input"] = input;

    std::string request_body = request_json.dump();

    auto res = this->client->Post("/v1/responses", request_body, "application/json");

    if (!res)
    {
        std::cerr << "❌ LLM request failed: null response" << std::endl;
        throw std::runtime_error("LLM request failed: null response");
    }

    std::cout << "✅ HTTP status: " << res->status << std::endl;
    std::cout << "📦 Response body:\n"
              << res->body << std::endl;

    if (res->status == 200)
    {
        json response_json = json::parse(res->body);
        return response_json["choices"][0]["text"];
    }
    // else
    // {
    //     std::cerr << "❌ LLM server returned non-200 status: " << res->status << std::endl;
    //     throw std::runtime_error("LLM request failed: server error");
    // }
}