#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>

#include <pcl_msgs/ModelCoefficients.h>   // <<< NEW

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

// normal estimation stage 
class NeStage{
public:
  NeStage (){}
  void apply(const CloudT::ConstPtr& in, pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals){
    pcl::NormalEstimation<PointT, pcl::Normal>::Ptr ne (new pcl::NormalEstimation<PointT, pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    ne->setSearchMethod (tree);
    ne->setInputCloud (in);
    // ne->setRadiusSearch(0.1f);
    ne->setKSearch (100);
    ne->compute (*cloud_normals);
  }
};

// sacseg from normals
class RansacStage{
public:
  RansacStage(){}
  // <<< CHANGED: add coeffs_out param
  void apply (const CloudT::ConstPtr& in,
              const pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals,
              CloudT::Ptr& out,
              pcl::ModelCoefficients::Ptr& coeffs_out) {
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    // seg.setMethodType(pcl::SAC_MSAC);   // or pcl::SAC_RMSAC
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.2f);
    seg.setMaxIterations (100000);
    // seg.setProbability(0.999f);
    seg.setDistanceThreshold (0.1);
    seg.setRadiusLimits (0.1, 1);
    seg.setInputCloud (in);
    seg.setInputNormals (cloud_normals);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients);
    seg.segment(*inliers, *coeffs);

    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(in);
    ex.setIndices(inliers);
    ex.setNegative(false);
    if (!out) out.reset(new CloudT);
    ex.filter(*out);

    // <<< NEW: hand coefficients back up
    if (!coeffs_out) coeffs_out.reset(new pcl::ModelCoefficients);
    *coeffs_out = *coeffs;
  }
};

class PipelineNode{
public:
  PipelineNode (ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh){
    normal_est_.reset(new NeStage());
    ransac_.reset(new RansacStage());

    sub_      = nh_.subscribe("/cluster_cloud", 1, &PipelineNode::cb, this);
    pub_cyl_  = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_cylinder", 1);
    pub_coef_ = nh_.advertise<pcl_msgs::ModelCoefficients>("/cylinder_coeffs", 1);  // <<< NEW
    pub_centroid_ = nh_.advertise<geometry_msgs::PoseArray>("/cylinder_centroids", 1);
  }

private:
  void cb(const sensor_msgs::PointCloud2ConstPtr& msg){
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);
    std::vector<int> drop; pcl::removeNaNFromPointCloud(*cloud, *cloud, drop);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    CloudT::Ptr cloud_out (new CloudT);
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);  // <<< NEW

    normal_est_->apply(cloud,  cloud_normals);
    ransac_->apply(cloud, cloud_normals, cloud_out, coeffs);         // <<< CHANGED

    if (!cloud_out || cloud_out->empty()) { ROS_WARN("no cylinder inliers"); return; }

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(*cloud_out, out_msg);
    out_msg.header = msg->header;
    pub_cyl_.publish(out_msg);

    // <<< NEW: publish cylinder coefficients for downstream marker node
    pcl_msgs::ModelCoefficients mc;
    mc.header = msg->header;
    mc.values = coeffs->values;   // (px,py,pz, ax,ay,az, r)
    pub_coef_.publish(mc);

    ROS_INFO_STREAM("in=" << cloud->size() << " cyl=" << cloud_out->size());


    //publish the centroid of the cylinder inlier points
    Eigen::Vector4f c;
    pcl::compute3DCentroid(*cloud_out, c);
    geometry_msgs::PoseArray arr;
    arr.header = msg->header;
    geometry_msgs::Pose P;
    P.position.x = c[0];
    P.position.y = c[1];
    P.position.z = c[2];
    P.orientation.w = 1.0;
    arr.poses.push_back(P);
    pub_centroid_.publish(arr);
  }

  std::shared_ptr<RansacStage> ransac_;
  std::shared_ptr<NeStage> normal_est_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_cyl_;
  ros::Publisher pub_coef_;   
  ros::Publisher pub_centroid_;
};

int main (int argc, char** argv){
  ros::init(argc, argv, "ransac_fit_node");
  ros::NodeHandle nh, pnh("~");
  PipelineNode node(nh,pnh);
  ros::spin();
  return 0;
}
