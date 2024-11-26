#ifndef SERVICE_HELPERS_H
#define SERVICE_HELPERS_H

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/execute_motion.hpp"
#include <optional>

class MotionServiceClient {
public:
    MotionServiceClient(rclcpp::Node::SharedPtr node, const std::string &service_name)
        : node_(node), client_(node_->create_client<custom_interfaces::srv::ExecuteMotion>(service_name)) {}

    void sendRequest(const std::string &motion_name) {
        auto request = std::make_shared<custom_interfaces::srv::ExecuteMotion::Request>();
        request->motion_name = motion_name;

        // Send the request and store the FutureAndRequestId in optional
        future_and_request_id_ = client_->async_send_request(request);
        response_received_ = false;
    }

    bool isServiceComplete() {
        if (future_and_request_id_ && future_and_request_id_->future.valid()) {
            auto status = rclcpp::spin_until_future_complete(node_, future_and_request_id_->future, std::chrono::milliseconds(10));
            if (status == rclcpp::FutureReturnCode::SUCCESS) {
                response_received_ = true;
                response_success_ = future_and_request_id_->future.get()->success;
            }
        }
        return response_received_;
    }

    bool getResponseSuccess() const { return response_success_; }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<custom_interfaces::srv::ExecuteMotion>::SharedPtr client_;
    std::optional<rclcpp::Client<custom_interfaces::srv::ExecuteMotion>::FutureAndRequestId> future_and_request_id_;
    bool response_received_ = false;
    bool response_success_ = false;
};

#endif  // SERVICE_HELPERS_H
