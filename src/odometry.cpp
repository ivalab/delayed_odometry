// #include <delayed_odometry/latency_injector.h>
#include <delayed_odometry/odom_latency_injector.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>
#include <ros/ros.h>

// template <>
// inline
// const ros::Time& getTimestamp(const tf2_msgs::TFMessage& t) {return t.transforms[0].header.stamp;}

namespace ros
{
  namespace message_traits
  {

    template <>
    struct TimeStamp<tf2_msgs::TFMessage>
    {
      static ros::Time* pointer(tf2_msgs::TFMessage& m)
      {
        return &m.transforms[0].header.stamp;
      }
      
      static ros::Time const* pointer(const tf2_msgs::TFMessage& m)
      {
        return &m.transforms[0].header.stamp;
      }
      
      static ros::Time value(const tf2_msgs::TFMessage& m)
      {
        return m.transforms[0].header.stamp;
      }
      
    };
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_latency_injector");
  
  float delay=.05;
  float rate = 100;
  
  ros::NodeHandle pnh("~");
  if(pnh.getParam("delay", delay))
  {
    ROS_INFO_STREAM("Delay = " << delay);
  }
  if(pnh.getParam("rate", rate))
  {
    ROS_INFO_STREAM("Rate = " << rate);
  }

  OdomLatencyInjector odom_latency(delay, rate, "odom_sparse", "delayed_pose");
  odom_latency.init(delay, rate);

  // LatencyInjector<nav_msgs::Odometry> odom_latency(delay, rate, "odom", "delayed_odom");
  // odom_latency.init(delay, rate);

  // LatencyInjector<tf2_msgs::TFMessage> tf_latency(delay, rate, "tf", "delayed_tf");
  // tf_latency.init(delay, rate);
  
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  
}
