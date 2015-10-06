#include <ros/ros.h>
#include <map>
#include <vector>
#include <pcl_clustering/Clusters.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Point.h>
#include "ford_msgs/PedTraj.h"
#include "ford_msgs/PedTrajVec.h"
#include "ford_msgs/Pose2DStamped.h"


class PedTrajData{
public:
    bool isPed_;
    ros::Time lastUpdateTime_;
    std::vector<ford_msgs::Pose2DStamped> traj_;
    std::vector<ford_msgs::Pose2DStamped>::iterator trajIter_;

    PedTrajData(){
        isPed_ = false;
        lastUpdateTime_ = ros::Time::now();
        trajIter_ = traj_.begin();
    }
    ~PedTrajData(){}

    void addData(const std_msgs::Header& header, const geometry_msgs::Point& point)
    {
        ford_msgs::Pose2DStamped pose2DStamped;
        pose2DStamped.header.frame_id = header.frame_id;
        pose2DStamped.header.stamp = header.stamp;
        pose2DStamped.pose.x = point.x;
        pose2DStamped.pose.y = point.y;
        lastUpdateTime_ = header.stamp;
        traj_.push_back(pose2DStamped);
    }
    void setPed(){isPed_ = true;}

    std::vector<ford_msgs::Pose2DStamped> getPoseVec(bool diff_only)
    {
        if (diff_only){
            return std::vector<ford_msgs::Pose2DStamped>(trajIter_,traj_.end());         
        }
        else{
            return traj_;
        }
    }

    void updateDiffPtr(){
        trajIter_ = traj_.end();
    }

    bool hasDiff(){
        return trajIter_ != traj_.end();
    }

};


class PedManager{
public:
    ros::NodeHandle nh_p_;
    ros::Subscriber sub_clusters_;
    ros::Subscriber sub_ped_id_;
    ros::Publisher pub_ped_diff_;

    ros::Timer timer_ped_diff_;

    ros::Time current_cluster_time_;

    std::map<size_t, PedTrajData> ped_map_;

    double diff_pub_time;

    PedManager()
    {
        nh_p_ = ros::NodeHandle("~");
        // TODO set and read parameters
        sub_clusters_ = nh_p_.subscribe("clusters",10,&PedManager::cbClusters,this);
        sub_ped_id_ = nh_p_.subscribe("ped_id",10,&PedManager::cbPedId,this);
        pub_ped_diff_ = nh_p_.advertise<ford_msgs::PedTrajVec>("ped_diff",1,true); //true for latching

        diff_pub_time = 1.0;
        timer_ped_diff_= nh_p_.createTimer(ros::Duration(diff_pub_time),&PedManager::cbPublishPedDiff,this);
    }
    ~PedManager(){}

    void cbClusters(const pcl_clustering::Clusters& clusters){
        // Manage the clusers
        current_cluster_time_ = clusters.header.stamp; //Use the cluster time to avoid time synce problem between machines
        for (int i = 0; i < clusters.labels.size(); i++){
            // This will create new map entry it's not there already
            ped_map_[clusters.labels[i]].addData(clusters.header,clusters.mean_points[i]);
        }
    }
    void cbPedId(const std_msgs::UInt32 ped_id)
    {
        std::map<size_t, PedTrajData>::iterator it = ped_map_.find(ped_id.data);
        if (it != ped_map_.end()){
            it->second.setPed();
        }
        // Do nothing if the ped_id is not in the map
    }

    std::vector<ford_msgs::PedTraj> getPedTrajVec(bool diff_only, bool ped_only)
    {
        // Return a vector of PedTraj messages
        std::vector<ford_msgs::PedTraj> pedTrajVec;
        std::map<size_t, PedTrajData>::iterator it;
        ros::Time current_time = ros::Time::now();

        for (it = ped_map_.begin(); it != ped_map_.end(); ++it){
            if (ped_only){
                if (not it->second.isPed_){
                    continue;
                    // Skip non-pedestrian 
                }
            }

            if (diff_only){
                if (it->second.hasDiff()){
                    // Only publish if has diff.
                    pedTrajVec.push_back(toPedTraj(it->first,current_time,it->second.getPoseVec(diff_only)));
                }
            }
            else{
                // Publish all
                pedTrajVec.push_back(toPedTraj(it->first,current_time,it->second.getPoseVec(diff_only)));    
            }
        }
        return pedTrajVec;
    }

    ford_msgs::PedTraj toPedTraj(size_t ped_id, ros::Time current_time, const std::vector<ford_msgs::Pose2DStamped>& poseVec){
        ford_msgs::PedTraj pedTrajMsg;
        pedTrajMsg.ped_id = ped_id;
        if (poseVec.size() > 0){
            pedTrajMsg.header.frame_id = poseVec[0].header.frame_id;
        }
        pedTrajMsg.header.stamp = current_time;
        pedTrajMsg.traj = poseVec;
        return pedTrajMsg;
    }

    void cbPublishPedDiff(const ros::TimerEvent& timerEvent){
        ford_msgs::PedTrajVec ped_traj_vec_msg;
        ped_traj_vec_msg.ped_traj_vec = getPedTrajVec(true,true);

        // Update the Diff Pointer
        std::map<size_t, PedTrajData>::iterator it;
        for (it = ped_map_.begin(); it != ped_map_.end(); ++it){
            it->second.updateDiffPtr();
        }
        // Publish
        pub_ped_diff_.publish(ped_traj_vec_msg);
    }

};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ped_manager");
    PedManager ped_manager;
    ros::spin();
    return 0;
}