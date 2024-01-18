
//#include <message_filters/time_sequencer.h>
#include <message_filters/subscriber.h>
#include <delayed_odometry/throttled_sequencer.h>


template <typename M>
class LatencyInjector
{
  private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<M> sub_;
    boost::shared_ptr<message_filters::ThrottledTimeSequencer<M> > sequencer_;
    ros::Publisher pub_, pose_pub_;
    int queue_size_ = 1000;
    std::string in_topic_, out_topic_;
    float delay_, rate_;
  
  public:
    LatencyInjector(float delay=.05, float rate=1000, std::string in_topic="in", std::string out_topic="out") :
      delay_(delay),
      rate_(rate),
      in_topic_(in_topic),
      out_topic_(out_topic)
    {
      init();
    }

    bool init()
    {
      pub_ = nh_.advertise<M>(out_topic_,queue_size_);
      sub_.subscribe(nh_, in_topic_, queue_size_);
      sequencer_ = boost::make_shared<message_filters::ThrottledTimeSequencer<M> >(sub_, ros::Duration(delay_), ros::Duration(1/rate_), queue_size_, nh_);
      sequencer_->registerCallback(boost::bind(&LatencyInjector::callback, this, _1));
      return true;
    }

  private:

    void callback(const typename M::ConstPtr& m)
    {
      pub_.publish(m);
    }

};
