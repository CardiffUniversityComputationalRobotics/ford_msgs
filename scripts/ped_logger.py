#!/usr/bin/env python
import rospy
import time, sys
# from os import path, makedirs
import rospkg


from ford_msgs.msg import Pose2DStamped, PedTraj, PedTrajVec

class PedLogger(object):
    def __init__(self):

        rospack = rospkg.RosPack()
        # self.directory = rospy.get_param('~output_file_directory', '')
        self.directory = rospack.get_path('ford_msgs') + "/log"
        self.file_name = self.directory+'/trajectories_'+time.strftime('%m-%d-%Y_%H:%M:%S')+'.txt'
        self.sub_ped_diff = rospy.Subscriber("~ped_diff",PedTrajVec, self.cbPedDiff)
        rospy.loginfo("[PedLogger] Logging to " + self.file_name)

        # Open file and prepare to write
        # if not path.exists(self.directory):
        #     makedirs(self.directory)
        with open(self.file_name, 'w') as f:
            f.write('time, cluster_id, x, y, z \n')

    def cbPedDiff(self,ped_traj_vec):
        for ped_traj in ped_traj_vec.ped_traj_vec:
            ped_id = ped_traj.ped_id
            with open(self.file_name, 'a') as f:
                for poseStamped in ped_traj.traj:
                    f.write(str([poseStamped.header.stamp.to_sec(), ped_id, poseStamped.pose.x, poseStamped.pose.y, 0.0]).strip('[]')+'\n')

if __name__ == '__main__':
    rospy.init_node('ped_logger',anonymous=False)
    ped_logger = PedLogger()
    rospy.spin()