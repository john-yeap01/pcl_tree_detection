// Isolation test node to determine if the cropbox is in the right orientation (w.r.t. drone)
// To solve flickering take last tf
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/exceptions.h>  // <-- already added

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class Stage {
public:
  virtual void apply(const CloudT::ConstPtr& in, CloudT::Ptr& out) = 0;
  virtual ~Stage() {}
};

class CropboxStage : public Stage {
public:
  CropboxStage(float x_near, float x_far, float y_near, float y_far, float z_min, float z_max)
  : min_B_(x_near, y_near, z_min, 1.0f), max_B_(x_far, y_far, z_max, 1.0f),
    A_WB_(Eigen::Affine3f::Identity()) {}

  void setWorldToBody(const Eigen::Affine3f& A_WB) { A_WB_ = A_WB; }

  void setExtents(float x_near, float x_far,
                  float y_near, float y_far,
                  float z_min,  float z_max)
  {
    min_B_ = Eigen::Vector4f(x_near, y_near, z_min, 1.0f);
    max_B_ = Eigen::Vector4f(x_far,  y_far,  z_max,  1.0f);
  }

  void apply(const CloudT::ConstPtr& in, CloudT::Ptr& out) override {
    if (!in || in->empty()) { out->clear(); return; }

    pcl::CropBox<PointT> box;
    box.setInputCloud(in);
    box.setMin(min_B_);
    box.setMax(max_B_);
    box.setTransform(A_WB_);
    box.setNegative(false);

    box.filter(*out);
    out->width = out->size();
    out->height = 1;
    out->is_dense = true;
  }

private:
  Eigen::Vector4f  min_B_;
  Eigen::Vector4f  max_B_;
  Eigen::Affine3f  A_WB_;
};

class Node {
public:
  Node(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  // ---- MINIMAL CHANGE 1: give TF buffer a deeper cache ----
  : nh_(nh), pnh_(pnh), buffer_(ros::Duration(10.0)), tf_listener_(buffer_) {
    pnh_.param<std::string>("target_frame", target_frame_, std::string("base_link"));

    box_.reset(new CropboxStage(0, 10, -5, 5, -3, 10));

    sub_cloud_   = nh_.subscribe("/fsm_high/fsm_node_high/rog_map/occ", 1, &Node::cb, this);
    pub_cropped_ = nh_.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, false);

    // ---- MINIMAL CHANGE 2: init last-known transform ----
    A_WB_last_.setIdentity();
    have_tf_ = false;

    ROS_INFO_STREAM("Cropbox target_frame=" << target_frame_);
  }

private:
  void cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    const std::string source = msg->header.frame_id;

    // ---- MINIMAL CHANGE 3: prefer latest TF, reuse last-good on failure ----
    try {
      // Use latest transform available (Time(0)) with small timeout
      geometry_msgs::TransformStamped T =
          buffer_.lookupTransform(target_frame_, source, ros::Time(0), ros::Duration(0.05));

      const Eigen::Isometry3d A_WB_d = tf2::transformToEigen(T);
      A_WB_last_ = Eigen::Affine3f(A_WB_d.cast<float>().matrix());
      have_tf_ = true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "TF lookup failed (%s <- %s): %s",
                        target_frame_.c_str(), source.c_str(), ex.what());
      if (!have_tf_) {
        // first frames: nothing to reuse yet
        return;
      }
      // else: fall through and reuse A_WB_last_
    }

    box_->setWorldToBody(A_WB_last_);

    CloudT::Ptr in(new CloudT), out(new CloudT);
    pcl::fromROSMsg(*msg, *in);
    box_->apply(in, out);

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*out, msg_out);

    // ---- MINIMAL CHANGE 4: keep frame, but refresh stamp to "now" ----
    msg_out.header.frame_id = msg->header.frame_id;
    msg_out.header.stamp = ros::Time::now();
    // msg_out.header.stamp = msg->header.stamp;



    pub_cropped_.publish(msg_out);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::shared_ptr<CropboxStage> box_;

  ros::Subscriber sub_cloud_;
  ros::Publisher  pub_cropped_;

  // order matters: buffer_ before tf_listener_
  tf2_ros::Buffer            buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ---- MINIMAL CHANGE 5: cache last transform ----
  Eigen::Affine3f A_WB_last_;
  bool            have_tf_;

  std::string target_frame_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cropbox_sanity");
  ros::NodeHandle nh, pnh("~");
  Node node(nh, pnh);
  ros::spin();
  return 0;
}
