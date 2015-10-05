#include <ros/ros.h>
#include <map>
#include <vector>
#include <pcl_clustering/Clusters.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Point.h>
#include "ford_msgs/PedTraj.h"
#include "ford_msgs/Pose2DStamped.h"


class PedTrajData{
public:
    bool isPed_;
    ros::Time lastUpdateTime_;
    std::vector<ford_msgs::Pose2DStamped> traj_;

    PedTrajData(){
        isPed_ = false;
        lastUpdateTime_ = ros::Time::now();
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

    ford_msgs::PedTraj toPedTraj(size_t ped)
    {
        ford_msgs::PedTraj pedTrajMsg;
        pedTrajMsg.header.frame_id = traj_[0].header.frame_id; //TODO handle empty traj
        pedTrajMsg.header.stamp = ros::Time::now();
        pedTrajMsg.traj = traj_;
        return pedTrajMsg;
    }
};


class PedManager{
public:
    ros::NodeHandle nh_p_;
    ros::Subscriber sub_clusters_;
    ros::Subscriber sub_ped_id_;
    std::map<size_t, PedTrajData> ped_map_;

    PedManager()
    {
        nh_p_ = ros::NodeHandle("~");
        // TODO set and read parameters

        sub_clusters_ = nh_p_.subscribe("clusters",10,&PedManager::cbClusters,this);
        sub_ped_id_ = nh_p_.subscribe("ped_id",10,&PedManager::cbPedId,this);
    }
    ~PedManager(){}

    // PedTrajData* getPedTrajData(size_t key)
    // {
    //     std::map<size_t, PedTrajData>::iterator it = ped_map_.find(key);
    //     if (it != ped_map_::end()){
    //         return &it->second;
    //     }
    //     else{
    //         return ped_map_[key];
    //     }
    // }
    void cbClusters(const pcl_clustering::Clusters& clusters){
        // Manage the clusers
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

    std::vector<ford_msgs::PedTraj> getPedTrajVec()
    {
        std::vector<ford_msgs::PedTraj> pedTrajVec;
        std::map<size_t, PedTrajData>::iterator it;
        for (it = ped_map_.begin(); it != ped_map_.end(); ++it){
            pedTrajVec.push_back(it->second.toPedTraj(it->first));
        }
    }

};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ped_manager");
    PedManager ped_manager;
    ros::spin();
    return 0;
}