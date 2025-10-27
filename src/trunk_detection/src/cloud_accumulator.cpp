// Takes in cloud registered direct from fast lio 
// to accumulate
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <deque>

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

class CloudAccumulator {
public:
  CloudAccumulator(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("cloud_topic", cloud_topic_, std::string("/cloud_registered"));
    pnh.param<double>("buffer_duration", buffer_duration_, 3.0);   // seconds
    pnh.param<double>("publish_rate", publish_rate_, 1.0);         // Hz

    sub_ = nh.subscribe(cloud_topic_, 5, &CloudAccumulator::cb, this);
    pub_ = nh.advertise<sensor_msgs::PointCloud2>("accumulated_cloud", 1);

    timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_),
                            &CloudAccumulator::publishAccumulated, this);

    ROS_INFO_STREAM("Accumulating from " << cloud_topic_
                    << " for last " << buffer_duration_ << " sec.");
  }

private:
  void cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);

    buffer_.push_back({msg->header.stamp, cloud});

    // drop old clouds
    ros::Time now = ros::Time::now();
    while (!buffer_.empty() &&
           (now - buffer_.front().first).toSec() > buffer_duration_) {
      buffer_.pop_front();
    }
  }

  void publishAccumulated(const ros::TimerEvent&) {
    if (buffer_.empty()) return;

    CloudT::Ptr merged(new CloudT);
    merged->header.frame_id = buffer_.back().second->header.frame_id;

    for (auto& pair : buffer_) {
      *merged += *pair.second;
    }

    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(*merged, out);
    out.header.stamp = ros::Time::now();
    pub_.publish(out);

    ROS_DEBUG_STREAM("Published accumulated cloud with "
                     << merged->size() << " points.");
  }

  std::string cloud_topic_;
  double buffer_duration_, publish_rate_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Timer timer_;

  std::deque<std::pair<ros::Time, CloudT::Ptr>> buffer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_accumulator");
  ros::NodeHandle nh, pnh("~");
  CloudAccumulator node(nh, pnh);
  ros::spin();
  return 0;
}
