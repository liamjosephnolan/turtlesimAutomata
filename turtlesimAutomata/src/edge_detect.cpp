// include all of our necessary libraries
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <random>

// define namespace
using namespace std::chrono_literals;

// this function starts the turtle at a random angle in the center of the screen. It uses the teleport_absolute service
void teleportTurtle(const rclcpp::Node::SharedPtr& node) {

    //define our service node
    auto turtleTeleport = node->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");

    // state our x and y teleport target
    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    request->x = 5.5;
    request->y = 5.5;

    // Generate a random initial angle between 0 and 360 degrees
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0, 2 * M_PI); // Define the range for angle in radians
    request->theta = dis(gen); // Set the random angle

    // run the service and output whether it was successful or not
    while (!turtleTeleport->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }

    auto result = turtleTeleport->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to teleport turtle");
    } else {
        RCLCPP_INFO(node->get_logger(), "Turtle teleportation completed");
    }
}

// this is our edge detection function, it relies on the x and y postiton of the turtle to determine if it is inbounds or not
void pose_callback(const turtlesim::msg::Pose::SharedPtr msg, const rclcpp::Node::SharedPtr node,
                   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher) {
    static bool is_rotating = false; // Flag to indicate if the turtle is rotating
    static float initial_angle = 0.0; // Static variable to store the initial angle

    // Check if the turtle is near the edge
    if (msg->x <= 0 || msg->x >= 11 || msg->y <= 0 || msg->y >= 11) {
        // If the turtle is not already rotating, initiate rotation
        if (!is_rotating) {
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0; // Stop
            twist.angular.z = -5.0; // Rotate
            publisher->publish(twist);
            printf("Edge detected\n");
            initial_angle = msg->theta; // Store the initial angle
            is_rotating = true; // Set the flag to indicate rotation
        }
        // Check if the turtle has rotated 90 degrees from the initial angle
        if (std::abs(msg->theta - initial_angle) >= 1.570796) { // 90 degrees in radians
            // If rotation is completed, initiate forward movement
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 5; // Drive forward
            twist.angular.z = 0.0; // No rotation
            publisher->publish(twist); // Publish the move instruction to stop rotating 
            is_rotating = false; // Reset the flag
        }
    } else {
        // If the turtle is not near the edge, initiate forward movement
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 5; // Drive forward
        twist.angular.z = 0.0; // No rotation
        publisher->publish(twist);
        is_rotating = false; // Reset the flag
    }
}

bool initial_state = true; // boolian flag to determine if we are in our inital state

// main loop that will run our code
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    // start node
    auto node = rclcpp::Node::make_shared("edge_detector");

    // check to see if code has run already, if not we define starting postion for turtle
    if (initial_state == true){
        printf("Starting\n");
        teleportTurtle(node);  // start turtle with random thetea
        initial_state = false; // reset boolian flag
    }

    // publish twist 
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    
    // subscribe to turtle pose
    auto subscription = node->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, 
        [node, publisher](const turtlesim::msg::Pose::SharedPtr msg) {
            pose_callback(msg, node, publisher);
        });

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
