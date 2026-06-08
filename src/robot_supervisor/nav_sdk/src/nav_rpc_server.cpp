#include <asio.hpp>
#include <iostream>
#include <thread>
#include "nav_sdk.h"
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

using asio::ip::tcp;
using json = nlohmann::json;

class JsonRpcServer
{
public:
    JsonRpcServer(int port)
        : acceptor_(io_context_, tcp::endpoint(tcp::v4(), port))
    {
        nav_ = std::make_shared<NavigationSDK>();
        std::thread([this]() { rclcpp::spin(nav_); }).detach();
    }

    void start()
    {
        while (true)
        {
            tcp::socket socket(io_context_);
            acceptor_.accept(socket);

            std::thread(&JsonRpcServer::handleClient, this, std::move(socket)).detach();
        }
    }

private:
    asio::io_context io_context_;
    tcp::acceptor acceptor_;
    std::shared_ptr<NavigationSDK> nav_;

    void handleClient(tcp::socket socket)
    {
        try
        {
            for (;;)
            {
                // char data[1024];
                // size_t len = socket.read_some(asio::buffer(data));

                // std::string req_str(data, len);
                asio::streambuf buf;
                asio::read_until(socket, buf, "\n");

                std::istream is(&buf);
                std::string req_str;
                std::getline(is, req_str);

                auto req = json::parse(req_str);

                json res;

                res["jsonrpc"] = "2.0";
                res["id"] = req["id"];

                std::string method = req["method"];

                RCLCPP_INFO(rclcpp::get_logger("JsonRpcServer"), "RPC received: %s", method.c_str());

                if (method == "addGoal")
                {
                    auto p = req["params"];
                    nav_->addGoal(p["x"], p["y"], p["yaw"]);
                    res["result"] = true;
                }
                else if (method == "cancelAll")
                {
                    nav_->cancelAll();
                    res["result"] = true;
                }
                else if (method == "getState")
                {
                    res["result"] = {
                        {"state", (int)nav_->getState()},
                        {"queue_size", nav_->getQueueSize()}
                    };
                }
                else
                {
                    res["error"] = "Unknown method";
                }

                std::string res_str = res.dump()+"\n";
                asio::write(socket, asio::buffer(res_str));
            }
        }
        catch (...)
        {
            std::cout << "Client disconnected\n";
        }
    }
};

int main(int argc, char **argv)
{
    // 初始化 ROS
    rclcpp::init(argc, argv);

    try
    {
        int port = 12345;  // 你可以改成参数

        std::cout << "Starting JSON-RPC Navigation Server on port: " << port << std::endl;

        JsonRpcServer server(port);
        server.start();   // 阻塞运行
    }
    catch (const std::exception &e)
    {
        std::cerr << "Server error: " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}