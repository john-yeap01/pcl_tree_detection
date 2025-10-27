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

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

// normal estimation stage 
class NeStage{
    public:

        NeStage (){

        }

        void apply(const CloudT::ConstPtr& in, pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals){
            pcl::NormalEstimation<PointT, pcl::Normal>::Ptr ne (new pcl::NormalEstimation<PointT, pcl::Normal>);
            pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
            // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;

            ne->setSearchMethod (tree);
            ne->setInputCloud (in);
            ne->setKSearch (50);
            ne->compute (*cloud_normals);
        }

    private:
        

};



// sacseg from normals
class RansacStage{
    public:
        RansacStage(){

        }


        void apply (const CloudT::ConstPtr& in, const pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals, CloudT::Ptr& out) {
            pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_CYLINDER);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight (0.1);
            seg.setMaxIterations (10000);
            seg.setDistanceThreshold (0.05);
            seg.setRadiusLimits (0, 0.1);
            seg.setInputCloud (in);
            seg.setInputNormals (cloud_normals);

            // Segment into the inlier indices
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients);
            seg.segment(*inliers, *coeffs);

            // Extract the indices out 
            pcl::ExtractIndices<PointT> ex;
            ex.setInputCloud(in);
            ex.setIndices(inliers);
            ex.setNegative(false);
            if (!out) out.reset(new CloudT);
            ex.filter(*out);

        }

    private:

        

};

class PipelineNode{
    public:
        PipelineNode (ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh){
            
            normal_est_.reset(new NeStage());
            ransac_.reset(new RansacStage());

            sub_ = nh_.subscribe("/cluster_cloud", 1, &PipelineNode::cb, this);
            pub_cyl_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_cylinder", 1);
        }


    private:

        // main node compute normals and then pass normals into ransac stage
        void cb(const sensor_msgs::PointCloud2ConstPtr& msg){
            // create objects 
            CloudT::Ptr cloud(new CloudT);
            pcl::fromROSMsg(*msg, *cloud);
            std::vector<int> drop; pcl::removeNaNFromPointCloud(*cloud, *cloud, drop);

            //Allocate output clouds for normals and output cloud 
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
            CloudT::Ptr cloud_out (new CloudT);

            // pass into the two processing Stages 
            normal_est_->apply(cloud,  cloud_normals);
            ransac_->apply(cloud, cloud_normals, cloud_out);

            if (!cloud_out || cloud_out->empty()) {ROS_WARN("no cylinder inliers"); return;}


            sensor_msgs::PointCloud2 out_msg;
            pcl::toROSMsg(*cloud_out, out_msg);
            out_msg.header = msg->header;
            pub_cyl_.publish(out_msg);

            // debug
            ROS_INFO_STREAM("in=" << cloud->size() << " cyl=" << cloud_out->size());
        }

        
        std::shared_ptr<RansacStage> ransac_;
        std::shared_ptr<NeStage> normal_est_;
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_;
        ros::Publisher pub_cyl_;

};


int main (int argc, char** argv){

    ros::init(argc, argv, "ransac_fit_node");
    ros::NodeHandle nh, pnh;
    PipelineNode node(nh,pnh);
    ros::spin();

    return 0;
}