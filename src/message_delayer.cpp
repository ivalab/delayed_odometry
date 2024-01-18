/**
 * @file messages.cpp
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 01-18-2024
 * @copyright Copyright (c) 2024
 */

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <memory>

#include <delayed_odometry/latency_injector.h>

class MessageDelayer
{
public:
    using TwistLatencyInjectorType = LatencyInjector<geometry_msgs::Twist>;
    using ImageLatencyInjectorType = LatencyInjector<sensor_msgs::Image>;
    using ImuLatencyInjectorType = LatencyInjector<sensor_msgs::Imu>;

    using TwistPtr = std::shared_ptr<TwistLatencyInjectorType>;
    using ImagePtr = std::shared_ptr<ImageLatencyInjectorType>;
    using ImuPtr = std::shared_ptr<ImuLatencyInjectorType>;

    /**
     * @brief Construct a new Message Delayer object
     *
     * @param delay
     * @param rate
     */
    MessageDelayer(double delay, double rate) : delay_(delay), rate_(rate)
    {
        init();
    }

private:
    void init()
    {
        std::string left_cam_topic  = "/multisense_sl/camera/left/image_raw";
        std::string right_cam_topic = "/multisense_sl/camera/right/image_raw";
        // std::string twist_topic     = "/cmd_vel_mux/input/navi";
        std::string imu_topic = "/imu0";

        left_cam_ = std::make_shared<ImageLatencyInjectorType>(
            delay_, rate_, left_cam_topic, left_cam_topic + "_delayed");
        right_cam_ = std::make_shared<ImageLatencyInjectorType>(
            delay_, rate_, right_cam_topic, right_cam_topic + "_delayed");
        // twist_ = std::make_shared<TwistLatencyInjectorType>(
        //     delay_, rate_, twist_topic, twist_topic + "_delayed");
        imu_ = std::make_shared<ImuLatencyInjectorType>(
            delay_, rate_, imu_topic, imu_topic + "_delayed");
    }

    double delay_;
    double rate_;

    ImagePtr left_cam_;
    ImagePtr right_cam_;
    TwistPtr twist_;
    ImuPtr   imu_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "message_delayer");

    double delay = .05;
    double rate  = 100;

    ros::NodeHandle pnh("~");
    if (pnh.getParam("delay", delay))
    {
        ROS_INFO_STREAM("Delay = " << delay);
    }
    if (pnh.getParam("rate", rate))
    {
        ROS_INFO_STREAM("Rate = " << rate);
    }
    MessageDelayer    message_delayer(delay, rate);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
}