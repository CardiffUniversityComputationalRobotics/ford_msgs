#include <map>
#include <vector>
#include <stdlib.h>
#include <cstdio>
#include <ctime>

#include <ros/ros.h>
// #include <rosbag/bag.h>
// #include <ros/package.h>

#include <pcl_clustering/Clusters.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "ford_msgs/PedTraj.h"
#include "ford_msgs/PedTrajVec.h"
#include "ford_msgs/Pose2DStamped.h"


class PedTrajData{
public:
    bool isPed_;
    ros::Time lastUpdateTime_;
    size_t ped_id_;
    std::string frame_id_;
    std::vector<ford_msgs::Pose2DStamped> traj_;
    int trajIndex_;

    PedTrajData(){
        isPed_ = false;
        lastUpdateTime_ = ros::Time::now();
        trajIndex_ = -1;
    }

    ~PedTrajData(){}

    void addData(const std_msgs::Header& header, size_t ped_id, const geometry_msgs::Point& point)
    {
        ped_id_ = ped_id;
        frame_id_ = header.frame_id;
        ford_msgs::Pose2DStamped pose2DStamped;
        pose2DStamped.header.frame_id = header.frame_id;
        pose2DStamped.header.stamp = header.stamp;
        pose2DStamped.pose.x = point.x;
        pose2DStamped.pose.y = point.y;
        lastUpdateTime_ = header.stamp;
        // std::cout << "[addData]: Before ped_id_: " << ped_id_  << " traj_.end() - trajIter_ " << trajIter_ - traj_.end() << std::endl;
        // std::cout << "[addData]: Before ped_id: " << ped_id_ << " traj_.size(): " << traj_.size() << " iter diff: " << trajIter_ - traj_.begin() << std::endl;
        traj_.push_back(pose2DStamped);
        // std::cout << "[addData]: After  ped_id: " << ped_id_ << " traj_.size(): " << traj_.size() << " iter diff: " << trajIter_ - traj_.begin() << std::endl;
    }
    void setPed(){isPed_ = true;}

    std::vector<ford_msgs::Pose2DStamped> getPoseVec(bool diff_only)
    {   
        // std::cout << "[getPoseVec] traj_.end() - trajIter_ = " << traj_.end() - trajIter_ << std::endl;
        if (diff_only){
            return std::vector<ford_msgs::Pose2DStamped>(traj_.begin() + (trajIndex_ + 1),traj_.end());
        }
        else{
            return traj_;
        }
    }

    void updateDiffIndex(){
        trajIndex_ = traj_.size() - 1;
    }

    bool hasDiff() const {
        return trajIndex_ < traj_.size() - 1;
    }

    ford_msgs::PedTraj toPedTraj(bool diff_only){
        ford_msgs::PedTraj pedTrajMsg;
        pedTrajMsg.ped_id = ped_id_;
        pedTrajMsg.header.frame_id = frame_id_;
        pedTrajMsg.header.stamp = lastUpdateTime_;
        pedTrajMsg.traj = getPoseVec(diff_only);
        return pedTrajMsg;
    }

};


class PedManager{
public:
    ros::NodeHandle nh_p_;
    ros::Subscriber sub_clusters_;
    ros::Subscriber sub_ped_id_;
    ros::Publisher pub_ped_diff_;
    ros::Publisher pub_markers_;
    ros::Publisher pub_ped_dump_;

    ros::Timer timer_ped_diff_;
    ros::Timer timer_prune_;
    ros::Timer timer_vis_;
    ros::Timer timer_dump_;

    ros::Time current_cluster_time_;

    std::map<size_t, PedTrajData*> ped_map_;
    std::map<size_t, std_msgs::ColorRGBA> color_map_;

    double diff_pub_period_;
    double prune_period_;
    double inactive_tol_;
    double vis_period_;
    double dump_period_;

    PedManager()
    {
        nh_p_ = ros::NodeHandle("~");
        setDefaultParameters();
        getParameters();

        sub_clusters_ = nh_p_.subscribe("clusters",10,&PedManager::cbClusters,this);
        sub_ped_id_ = nh_p_.subscribe("ped_id",10,&PedManager::cbPedId,this);
        pub_ped_diff_ = nh_p_.advertise<ford_msgs::PedTrajVec>("ped_diff",1,true); //true for latching
        pub_ped_dump_ = nh_p_.advertise<ford_msgs::PedTrajVec>("ped_dump",1,true); //true for latching
        pub_markers_ = nh_p_.advertise<visualization_msgs::MarkerArray>("ped_markers",1,true); //true for latching

        timer_ped_diff_= nh_p_.createTimer(ros::Duration(diff_pub_period_),&PedManager::cbPublishPedDiff,this);
        timer_prune_= nh_p_.createTimer(ros::Duration(prune_period_),&PedManager::cbPrune,this);
        timer_dump_ = nh_p_.createTimer(ros::Duration(dump_period_),&PedManager::cbDump,this);
        timer_vis_= nh_p_.createTimer(ros::Duration(vis_period_),&PedManager::cbVis,this);

    }
    ~PedManager(){}

    void setDefaultParameters()
    {
        if (!ros::param::has("~diff_pub_period")) { ros::param::set("~diff_pub_period",1.0);}
        if (!ros::param::has("~prune_period")) { ros::param::set("~prune_period",10.0);}
        if (!ros::param::has("~inactive_tol")) { ros::param::set("~inactive_tol",60.0);}
        if (!ros::param::has("~vis_period")) { ros::param::set("~vis_period",0.03);}
        if (!ros::param::has("~dump_period")) { ros::param::set("~dump_period",60.0);}
    }

    void getParameters()
    {
        ros::param::getCached("~diff_pub_period",diff_pub_period_);
        ros::param::getCached("~prune_period",prune_period_);
        ros::param::getCached("~inactive_tol",inactive_tol_);
        ros::param::getCached("~vis_period",vis_period_);
        ros::param::getCached("~dump_period",dump_period_);
    }



    void cbClusters(const pcl_clustering::Clusters& clusters){
        // Manage the clusers
        current_cluster_time_ = clusters.header.stamp; //Use the cluster time to avoid time synce problem between machines
        std::map<size_t, PedTrajData*>::iterator it;
        for (int i = 0; i < clusters.labels.size(); i++){
            it = ped_map_.find(clusters.labels[i]);
            if (it == ped_map_.end()){
                ped_map_[clusters.labels[i]] = new PedTrajData();
            }
            ped_map_[clusters.labels[i]]->addData(clusters.header,clusters.labels[i],clusters.mean_points[i]);
        }
    }

    void cbPedId(const std_msgs::UInt32 ped_id)
    {
        std::map<size_t, PedTrajData*>::iterator it = ped_map_.find(ped_id.data);
        if (it != ped_map_.end()){
            it->second->setPed();
        }
        // ROS_INFO_STREAM("[PedManager::cbPedId]:" << ped_id);
    }

    ford_msgs::PedTrajVec getPedTrajVec(bool diff_only, bool ped_only)
    {
        // Return a vector of PedTraj messages
        ford_msgs::PedTrajVec pedTrajVecMsg;
        std::map<size_t, PedTrajData*>::iterator it;
        ros::Time current_time = ros::Time::now();

        for (it = ped_map_.begin(); it != ped_map_.end(); ++it){
            if (ped_only){
                if (not it->second->isPed_){
                    continue; // Skip non-pedestrian 
                }
            }

            if (diff_only){
                if (it->second->hasDiff()){ // Only publish if has diff.
                    pedTrajVecMsg.ped_traj_vec.push_back(it->second->toPedTraj(diff_only));
                }
            }
            else{ // Publish all
                pedTrajVecMsg.ped_traj_vec.push_back(it->second->toPedTraj(diff_only));
            }
        }
        return pedTrajVecMsg;
    }

    void cbPublishPedDiff(const ros::TimerEvent& timerEvent)
    {
        ford_msgs::PedTrajVec ped_traj_vec_msg = getPedTrajVec(true,true);

        // Publish
        pub_ped_diff_.publish(ped_traj_vec_msg);
        // Update the Diff Pointer
        std::map<size_t, PedTrajData*>::iterator it;
        for (it = ped_map_.begin(); it != ped_map_.end(); ++it){
            it->second->updateDiffIndex();
        }
        ROS_INFO_STREAM(ped_traj_vec_msg);
    }

    void cbPrune(const ros::TimerEvent& timerEvent)
    {
        // pruneInactive(ros::Duration(inactive_tol_),true);
    }

    void pruneInactive(const ros::Duration& inactive_tol, bool keep_ped)
    {
        // ROS_INFO_STREAM("[PedManager::pruneInactive] Pruning started.");
        std::map<size_t, PedTrajData*>::iterator it;
        for (it = ped_map_.begin(); it != ped_map_.end();){
            if (keep_ped){
                if (it->second->isPed_){ // Don't prune pedestrian data
                    ++it;
                    continue;
                }
            }
            if (current_cluster_time_ - it->second->lastUpdateTime_ > inactive_tol){
                delete it->second;
                ped_map_.erase(it++);
            }
            else{
                ++it;
            }
        }
        // ROS_INFO_STREAM("[PedManager::pruneInactive] Pruning done.");
    }

    void cbDump(const ros::TimerEvent& timerEvent)
    {
        // ford_msgs::PedTrajVec pedTrajVec = popInactivePed(ros::Duration(inactive_tol_));
        // pub_ped_dump_.publish(pedTrajVec);
    }

    ford_msgs::PedTrajVec popInactivePed(const ros::Duration& inactive_tol)
    {
        // ROS_INFO_STREAM("[PedManager::popInactivePed] Popping started.");
        ford_msgs::PedTrajVec pedTrajVec;
        std::map<size_t, PedTrajData*>::iterator it;
        for (it = ped_map_.begin(); it != ped_map_.end();){
            if (it->second->isPed_){ //Only process if is pedestrian
                if (current_cluster_time_ - it->second->lastUpdateTime_ > inactive_tol){
                    pedTrajVec.ped_traj_vec.push_back(it->second->toPedTraj(false));
                    delete it->second;
                    ped_map_.erase(it++);
                }
                else{ //Skip if still active
                    ++it;
                }
            }
            else{ //Skip if not a pedestrian
                ++it;
            }
        }
        // ROS_INFO_STREAM("[PedManager::popInactivePed] Popping done.");
        return pedTrajVec;
    }

    void cbVis(const ros::TimerEvent& timerEvent)
    {
        // publishMarkers();
    }

    void publishMarkers()
    {
        // Publish whole trajectories of pedestrians
        visualization_msgs::MarkerArray markerArray;
        markerArray.markers = toMarkerVec(getPedTrajVec(false,true)); 
        pub_markers_.publish(markerArray);
    }

    std::vector<visualization_msgs::Marker> toMarkerVec(const ford_msgs::PedTrajVec& pedTrajVec)
    {
        std::vector<visualization_msgs::Marker> marker_vec;
        std::vector<ford_msgs::PedTraj>::const_iterator it;
        for (it = pedTrajVec.ped_traj_vec.begin(); it != pedTrajVec.ped_traj_vec.end(); ++it){
            std::vector<visualization_msgs::Marker> markers = toMarkerVec(*it);
            marker_vec.insert(marker_vec.end(),markers.begin(),markers.end());
        }
        return marker_vec;
    }

    std::vector<visualization_msgs::Marker> toMarkerVec(const ford_msgs::PedTraj& pedTraj)
    {
        // Convert PedTraj to a vector of Markers
        std::vector<visualization_msgs::Marker> marker_vec;
        size_t traj_length = pedTraj.traj.size();

        // TODO get color according to id

        // Current Position
        visualization_msgs::Marker pos_marker;
        pos_marker.header.frame_id = pedTraj.header.frame_id;
        pos_marker.header.stamp = pedTraj.header.stamp;
        pos_marker.ns = "ped_manager/pos";
        pos_marker.id = pedTraj.ped_id;
        pos_marker.lifetime = ros::Duration(vis_period_ + 0.2);
        pos_marker.type = visualization_msgs::Marker::CYLINDER;
        pos_marker.pose.orientation.w = 1.0;
        pos_marker.pose.position.x = pedTraj.traj.back().pose.x;
        pos_marker.pose.position.y = pedTraj.traj.back().pose.y;
        pos_marker.pose.position.z = 0.5;
        pos_marker.scale.x = 0.4;
        pos_marker.scale.y = 0.4;
        pos_marker.scale.z = 1.0;
        pos_marker.color = getColor(pedTraj.ped_id);
        marker_vec.push_back(pos_marker);

        // Trajectory
        visualization_msgs::Marker traj_marker;
        traj_marker.header.frame_id = pedTraj.header.frame_id;
        traj_marker.header.stamp = pedTraj.header.stamp;
        traj_marker.ns = "ped_manager/traj";
        traj_marker.id = pedTraj.ped_id;
        traj_marker.lifetime = ros::Duration(vis_period_ + 0.2);
        traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
        traj_marker.pose.orientation.w = 1.0;
        traj_marker.scale.x = 0.1;
        traj_marker.scale.y = 0.1;
        traj_marker.scale.z = 0.1;
        std::vector<ford_msgs::Pose2DStamped>::const_iterator it;
        for (it = pedTraj.traj.begin(); it != pedTraj.traj.end(); ++it){
            geometry_msgs::Point p;
            p.x = it->pose.x;
            p.y = it->pose.y;
            traj_marker.points.push_back(p);
        }
        traj_marker.color = getColor(pedTraj.ped_id);
        marker_vec.push_back(traj_marker);

        return marker_vec;
    }

    std_msgs::ColorRGBA getColor(size_t ped_id)
    {
        std::map<size_t, std_msgs::ColorRGBA>::iterator it = color_map_.find(ped_id);
        if (it == color_map_.end()){ // Insert random color for ped_id
            std_msgs::ColorRGBA color;
            color.r = ((float) rand()/(RAND_MAX));
            color.g = ((float) rand()/(RAND_MAX));
            color.b = ((float) rand()/(RAND_MAX));
            color.a = 1.0;
            color_map_[ped_id] = color;
        }
        return color_map_[ped_id];
    }

    std::string getTimeString(const ros::Time& ros_time)
    {
        // std::string time_str;
        // Test for time
        std::time_t rawtime;
        std::tm* timeinfo;
        char buffer [80];
        // std::time(&rawtime);
        rawtime = ros_time.sec;
        timeinfo = std::localtime(&rawtime);
        std::strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
        return std::string(buffer);
    }



};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ped_manager");
    PedManager ped_manager;
    ros::spin();
    return 0;
}