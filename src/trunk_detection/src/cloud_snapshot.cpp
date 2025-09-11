// file: cloud_snapshot.cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class CloudSnapshot
{
public:
  CloudSnapshot(ros::NodeHandle& nh)
  : nh_(nh)
  {
    // Params (with sensible defaults)
    nh_.param<std::string>("input",  input_topic_,  "/cloud_registered");
    nh_.param<std::string>("output", output_topic_, "/snapshot_cloud");
    nh_.param<double>("rate", publish_hz_, 5.0);
    nh_.param<bool>("latch", latch_, true);

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1, latch_);
    sub_ = nh_.subscribe(input_topic_, 1, &CloudSnapshot::cb, this);

    // timer will start publishing after we receive a snapshot
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, publish_hz_)),
                             &CloudSnapshot::tick, this, /*oneshot=*/false, /*autostart=*/false);

    ROS_INFO_STREAM("CloudSnapshot: waiting for first point cloud on " << input_topic_);
    ROS_INFO_STREAM("Will publish snapshot on " << output_topic_
                    << " at " << publish_hz_ << " Hz (latch=" << (latch_?"true":"false") << ")");
  }

private:
  void cb(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (got_) return;
    snapshot_ = *msg;     // store the first cloud as-is
    got_ = true;

    ROS_INFO_STREAM("CloudSnapshot: captured snapshot (" << snapshot_.width * snapshot_.height
                    << " points). Will publish continuously.");
    sub_.shutdown();      // we only need the first one
    timer_.start();       // start periodic publishing
  }

  void tick(const ros::TimerEvent&)
  {
    if (!got_) return;
    pub_.publish(snapshot_);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher  pub_;
  ros::Timer      timer_;

  std::string input_topic_, output_topic_;
  double publish_hz_{5.0};
  bool latch_{true};

  bool got_{false};
  sensor_msgs::PointCloud2 snapshot_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_snapshot");
  ros::NodeHandle nh("~");
  CloudSnapshot node(nh);
  ros::spin();
  return 0;
}
