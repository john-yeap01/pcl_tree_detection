#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/crop_box.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

// forward box extents in BODY frame (meters)
static constexpr float X_NEAR = 0.3f;   // start a bit ahead of the nose
static constexpr float X_FAR  = 10.0f;   // how far forward
static constexpr float Y_HALF = 10.0f;   // left/right half-width
static constexpr float Z_MIN  = -10.0f;  // below body origin
static constexpr float Z_MAX  =  10.0f;  // above body origin

// ---- da base class ----
class Stage {
public:
  virtual void apply(const CloudT::ConstPtr& in, CloudT::Ptr& out) = 0;
  virtual ~Stage() {}
};


// ---- Z slice ----
class PassthroughStage : public Stage {
public:
  PassthroughStage(float zmin, float zmax, bool negative=false)
  : zmin_(zmin), zmax_(zmax), negative_(negative) {}

  void apply(const CloudT::ConstPtr& in, CloudT::Ptr& out) override {
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(zmin_, zmax_);
    pass.setFilterLimitsNegative(negative_);
    pass.filter(*out);
  }
private:
  float zmin_, zmax_; bool negative_;
};

// ---- Euclidean clustering on slice ----
class ClusteringStage : public Stage {
public:
  ClusteringStage(float tol, int min_sz, int max_sz)
  : tolerance_(tol), min_sz_(min_sz), max_sz_(max_sz) {}

  void apply(const CloudT::ConstPtr& in, CloudT::Ptr& out) override {
    centroids_.clear();
    clusters_.clear();
    out->clear();
    if (!in || in->empty()) return;

    tree_->setInputCloud(in);

    std::vector<pcl::PointIndices> idxs;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(tolerance_);
    ec.setMinClusterSize(min_sz_);
    ec.setMaxClusterSize(max_sz_);
    ec.setSearchMethod(tree_);
    ec.setInputCloud(in);
    ec.extract(idxs);
    if (idxs.empty()) return;

    size_t best_idx = 0, best_sz = 0;
    clusters_.reserve(idxs.size());
    centroids_.reserve(idxs.size());

    // merge clusters into one point cloud 
    for (size_t i = 0; i < idxs.size(); ++i) {
      const auto& vi = idxs[i].indices;
      CloudT::Ptr c(new CloudT);
      c->reserve(vi.size());
      double sx=0.0, sy=0.0;
      for (int j : vi) { const auto& p = (*in)[j]; c->push_back(p); sx += p.x; sy += p.y; }
      c->width = c->size(); c->height = 1; c->is_dense = true;
      clusters_.push_back(c);

      const double n = static_cast<double>(vi.size());
      centroids_.emplace_back(static_cast<float>(sx/n), static_cast<float>(sy/n));
      if (vi.size() > best_sz) { best_sz = vi.size(); best_idx = i; }
    }

    // keep largest cluster as 'out' for quick debug
    *out = *clusters_[best_idx];
  }

  const std::vector<Eigen::Vector2f>& centroids() const { return centroids_; }
  const std::vector<CloudT::Ptr>&     clusters()  const { return clusters_; }

private:
  float tolerance_; int min_sz_, max_sz_;
  std::vector<Eigen::Vector2f> centroids_;
  std::vector<CloudT::Ptr>     clusters_;
  pcl::search::KdTree<PointT>::Ptr tree_{new pcl::search::KdTree<PointT>};
};

// ---- Node: slice + cluster, publish all clusters merged on one topic ----
class PipelineNode {
public:
  PipelineNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    // change the passthrough height here to crop out the tree leaves if necessary 
    passthrough_.reset(new PassthroughStage(0.2f, 10.0f, false));
    // passthrough_.reset(new PassthroughStage(-0.5f, 2.0f, false));

    clustering_.reset(new ClusteringStage(0.5f, 30, 10000));

    // sub_             = nh_.subscribe("/fsm_high/fsm_node_high/rog_map/occ", 1, &PipelineNode::cb, this);
    sub_             = nh_.subscribe("/cropped_cloud", 1, &PipelineNode::cb, this);

    pub_clusters_    = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_cloud", 1, false);
    pub_trunk_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/cluster_poses", 1);
  }

private:
  void cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (world_frame_ != msg->header.frame_id) world_frame_ = msg->header.frame_id;

    CloudT::Ptr full(new CloudT);
    pcl::fromROSMsg(*msg, *full);
    std::vector<int> drop; pcl::removeNaNFromPointCloud(*full, *full, drop);

    

    // 1) make the Z slice (apply after crop)
    CloudT::Ptr slice(new CloudT);
    passthrough_->apply(full, slice);
    ROS_INFO_STREAM("slice=" << slice->size());

    // 2) cluster the slice
    CloudT::Ptr largest(new CloudT);
    clustering_->apply(slice, largest);
    const auto& all = clustering_->clusters();
    const std::vector<Eigen::Vector2f> trunk_centroids = clustering_->centroids();
    ROS_INFO_STREAM("Number of clusters, centroids: " << all.size() << " " << trunk_centroids.size());

    // POSE ARRAY 
    geometry_msgs::PoseArray trunk_poses;
    trunk_poses.header.frame_id = msg->header.frame_id;
    trunk_poses.header.stamp = msg->header.stamp;

    for (const auto& c2d : trunk_centroids) {
      geometry_msgs::Pose pose;
      pose.position.x = c2d.x();
      pose.position.y = c2d.y();
      pose.position.z = 3.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      trunk_poses.poses.push_back(pose);
    }

    // These poses will be accepted by another node to transform into markers for vizualisation
    pub_trunk_poses_.publish(trunk_poses);

    // 3) merge ALL clusters into one PointXYZ cloud
    CloudT::Ptr merged(new CloudT);
    size_t total = 0; for (const auto& c : all) if (c) total += c->size();
    merged->reserve(total);
    for (const auto& c : all) {
      if (!c || c->empty()) continue;
      merged->insert(merged->end(), c->begin(), c->end());
    }
    merged->width = merged->size(); merged->height = 1; merged->is_dense = true;

    ROS_INFO_STREAM("[clusters] merged cloud points = " << merged->size());

    // 4) publish one topic with all clusters (no RGB)
    sensor_msgs::PointCloud2 m;
    pcl::toROSMsg(*merged, m);
    m.header = msg->header;
    pub_clusters_.publish(m);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher  pub_clusters_;
  ros::Publisher  pub_trunk_poses_;

  std::shared_ptr<PassthroughStage> passthrough_;
  std::shared_ptr<ClusteringStage>  clustering_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  std::string world_frame_ = "world";
  std::string body_frame_  = "body";
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trunk_slice_clusters_merged_node");
  ros::NodeHandle nh; ros::NodeHandle pnh("~");
  PipelineNode node(nh, pnh);
  ros::spin();
  return 0;
}
