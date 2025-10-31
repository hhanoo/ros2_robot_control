#include <rclcpp/rclcpp.hpp>

#include "ur_bridge_cpp/ur_class.hpp"

class URBridgeNode : public rclcpp::Node {
   public:
    URBridgeNode();
    ~URBridgeNode();

   private:
    URClass ur_;
};

URBridgeNode::URBridgeNode()
    : Node("ur_bridge_node") {
    RCLCPP_INFO(this->get_logger(), "UR Bridge Node started.");
}

URBridgeNode::~URBridgeNode() {
    RCLCPP_INFO(this->get_logger(), "UR Bridge Node shutting down.");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<URBridgeNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}