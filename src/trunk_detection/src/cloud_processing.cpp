#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

// ---- Stage base ----
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

    // keep largest in 'out' for quick debug if you want
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

// ---- Node: slice + cluster, publish ALL clusters merged on one topic ----
class PipelineNode {
public:
  PipelineNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    passthrough_.reset(new PassthroughStage(-0.5f, 5.0f, false));
    clustering_.reset(new ClusteringStage(0.5f, 30, 10000));

    sub_          = nh_.subscribe("/fsm_high/fsm_node_high/rog_map/occ", 1, &PipelineNode::cb, this);
    pub_clusters_ = nh_.advertise<sensor_msgs::PointCloud2>("/slice_clusters", 1, false);
  }

private:
  void cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    CloudT::Ptr full(new CloudT);
    pcl::fromROSMsg(*msg, *full);
    std::vector<int> drop; pcl::removeNaNFromPointCloud(*full, *full, drop);

    // 1) make the Z slice
    CloudT::Ptr slice(new CloudT);
    passthrough_->apply(full, slice);

    // 2) cluster the slice
    CloudT::Ptr largest(new CloudT);
    clustering_->apply(slice, largest);
    const auto& all = clustering_->clusters();

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
  std::shared_ptr<PassthroughStage> passthrough_;
  std::shared_ptr<ClusteringStage>  clustering_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trunk_slice_clusters_merged_node");
  ros::NodeHandle nh; ros::NodeHandle pnh("~");
  PipelineNode node(nh, pnh);
  ros::spin();
  return 0;
}
