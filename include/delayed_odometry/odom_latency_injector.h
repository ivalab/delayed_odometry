
//#include <message_filters/time_sequencer.h>
#include <message_filters/subscriber.h>
#include <delayed_odometry/throttled_sequencer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


// take odom message as input; publish delayed pose message instead
class OdomLatencyInjector
{
  private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<nav_msgs::Odometry> sub_;
    boost::shared_ptr<message_filters::ThrottledTimeSequencer<nav_msgs::Odometry> > sequencer_;
    ros::Publisher pose_pub_;
    int queue_size_ = 1000;
    std::string in_topic_, out_topic_;
    float delay_, rate_;
  
  public:
    OdomLatencyInjector(float delay=.05, float rate=1000, std::string in_topic="in", std::string out_topic="out") :
      delay_(delay),
      rate_(rate),
      in_topic_(in_topic),
      out_topic_(out_topic)
    {}
    
    bool init(float delay, float rate)
    {
      ROS_INFO_STREAM("delay = " << delay);
      ROS_INFO_STREAM("rate = " << rate);
      //
      pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(out_topic_,queue_size_);
      sub_.subscribe(nh_, in_topic_, queue_size_);
      sequencer_ = boost::make_shared<message_filters::ThrottledTimeSequencer<nav_msgs::Odometry> >(sub_, ros::Duration(delay), ros::Duration(1/rate), queue_size_, nh_);
      sequencer_->registerCallback(boost::bind(&OdomLatencyInjector::callback, this, _1));
      return true;
    }

  private:

    void callback(const nav_msgs::Odometry::ConstPtr& m)
    {
      geometry_msgs::PoseWithCovarianceStamped pose_msg;
      pose_msg.header = m->header;
      pose_msg.header.frame_id = "map";
      pose_msg.pose = m->pose;
      // convert m to pose message
      pose_pub_.publish(pose_msg);
    }

};
