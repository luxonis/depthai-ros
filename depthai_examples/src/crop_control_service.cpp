#include <ctype.h>
#include <stdio.h>

#include "depthai_ros_msgs/NormalizedImageCrop.h"
#include "ros/ros.h"

static constexpr float stepSize = 0.02;

void boundAdjuster(double& value) {
    if(value < 0) {
        std::cout << "values should always be greater than 0." << std::endl;
        std::cout << "Resetting to 0." << std::endl;
        value = 0;
    } else if(value > 1) {
        std::cout << "values should always be less than 1." << std::endl;
        std::cout << "Resetting to 1." << std::endl;
        value = 1;
    }
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "crop_control_service");
    ros::NodeHandle pnh("~");
    std::string serviceName = "test_srv";

    /*
    int badParams = 0;
    badParams += !pnh.getParam("service_name", serviceName);

    if (badParams > 0){
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find %d of the parameters");
    }
    */

    ros::ServiceClient client = pnh.serviceClient<depthai_ros_msgs::NormalizedImageCrop>(serviceName);
    depthai_ros_msgs::NormalizedImageCrop srvMsg;
    srvMsg.request.top_left.x = 0.2;
    srvMsg.request.top_left.y = 0.2;
    srvMsg.request.bottom_right.x = 0.2;
    srvMsg.request.bottom_right.y = 0.2;

    std::cout << "Use the following keys to control the cropping region" << std::endl;
    std::cout << "  Q/W -> Increment/Decrement the topleft X position" << std::endl;
    std::cout << "  A/S -> Increment/Decrement the topleft Y position" << std::endl;
    std::cout << "  E/R -> Increment/Decrement the bottomright X position" << std::endl;
    std::cout << "  D/F -> Increment/Decrement the bottomright Y position" << std::endl;
    std::cout << "  Preess ctrl+D to exit." << std::endl;
    char c;
    bool sendSignal = false;

    while(ros::ok()) {
        c = std::tolower(getchar());
        switch(c) {
            case 'w':
                srvMsg.request.top_left.x -= stepSize;
                boundAdjuster(srvMsg.request.top_left.x);
                sendSignal = true;
                break;
            case 'q':
                srvMsg.request.top_left.x += stepSize;
                boundAdjuster(srvMsg.request.top_left.x);
                sendSignal = true;
                break;
            case 'a':
                srvMsg.request.top_left.y += stepSize;
                boundAdjuster(srvMsg.request.top_left.y);
                sendSignal = true;
                break;
            case 's':
                srvMsg.request.top_left.y -= stepSize;
                boundAdjuster(srvMsg.request.top_left.y);
                sendSignal = true;
                break;
            case 'e':
                srvMsg.request.bottom_right.x += stepSize;
                boundAdjuster(srvMsg.request.bottom_right.x);
                sendSignal = true;
                break;
            case 'r':
                srvMsg.request.bottom_right.x -= stepSize;
                boundAdjuster(srvMsg.request.bottom_right.x);
                sendSignal = true;
                break;
            case 'd':
                srvMsg.request.bottom_right.y += stepSize;
                boundAdjuster(srvMsg.request.bottom_right.y);
                sendSignal = true;
                break;
            case 'f':
                srvMsg.request.bottom_right.y -= stepSize;
                boundAdjuster(srvMsg.request.bottom_right.y);
                sendSignal = true;
                break;
            default:
                std::cout << " Entered Invalid Key..!!!" << std::endl;
                std::cout << " Use the following keys to control the cropping region" << std::endl;
                std::cout << "  Q/W -> Increment/Decrement the topleft X position" << std::endl;
                std::cout << "  A/S -> Increment/Decrement the topleft Y position" << std::endl;
                std::cout << "  E/R -> Increment/Decrement the bottomright X position" << std::endl;
                std::cout << "  D/F -> Increment/Decrement the bottomright Y position" << std::endl;
                std::cout << "  Preess ctrl+D to exit." << std::endl;
        }

        if(sendSignal) {
            std::cout << "Top left Position -> (" << srvMsg.request.top_left.x << ", " << srvMsg.request.top_left.y << ")" << std::endl;
            std::cout << "Bottion right Position -> (" << srvMsg.request.bottom_right.x << ", " << srvMsg.request.bottom_right.y << ")" << std::endl;
            client.call(srvMsg);
            sendSignal = false;
        }
    }

    return 0;
}