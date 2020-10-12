#!/usr/bin/python

import os
import re
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point32
# from image_labeling_utils import artefact_gtConfig
from sensor_msgs.msg import ChannelFloat32
from sensor_msgs.msg import PointCloud

class Art():
    name = None
    position = None
    orientation = None
    label = None

class ArtGtPub():
    def __init__(self):

        self.is_initialized = False

        # #{ Load ROS parameters
        
        pub_rate =  rospy.get_param("~publish_rate", 10)
        self.sdf_path =  rospy.get_param("~sdf_path", "")
        self.object_keywords =  rospy.get_param("~objects_keywords", "")
        self.obj_frame =  rospy.get_param("~objects_frame", "")
        
        if self.sdf_path == "" or self.object_keywords == "":
            rospy.logerr('No SDF file specified, or objects are not specified, abort')
            rospy.signal_shutdown("SDF file or objects are not specified")
        
        if self.obj_frame == "":
            rospy.logerr('Frame is not specified, abort')
            rospy.signal_shutdown("Objects frame not specified")
        
        # #} end of Load ROS parameters
        
        # Parse the poses of the objects from the SDF file
        self.objects = self.parse_lines(self.sdf_path)
        if self.objects is False:
            rospy.logerr("No object found in SDF, abort")
            rospy.signal_shutdown("No object found in SDF")

        self.pub_arts_gt = rospy.Publisher("artefacts_gt", PoseArray, queue_size=10)
        self.pub_pcl_arts_gt = rospy.Publisher("pcl_artefacts_gt", PointCloud, queue_size=10)
        pub_timer = rospy.Timer(rospy.Duration(1.0/pub_rate), self.pub)

        rospy.loginfo('Node initialized')
        self.is_initialized = True
        

    def read_file(self,path):
        f = open(path, 'r')
        return f.readlines()


    def parse_lines(self,path):
        lines = self.read_file(path)
        artefacts = []
        for i in range(len(lines)):
            # if this line contains the tag "include"
            if lines[i].count("<include>") and i < len(lines)-3:
                # and one of the next three lines contains one of the keywords
                name = None
                pose = None
                for j in range(1, 4):
                    cname = re.findall('\<name\>(.*?)\</name\>', lines[i+j])
                    cpose = re.findall('\<pose\>(.*?)\</pose\>', lines[i+j])
                    if len(cname) > 0 and any(cname[0].count(kwd) for kwd in self.object_keywords):
                        name = cname[0]
                    if len(cpose) > 0:
                        pose = cpose[0].split(' ')
                        for j in range(0, len(pose)):
                            pose[j] = float(pose[j])
                if name is not None:
                    if pose is None or len(pose) != 6:
                        print('object "{}" found, but with invalid pose: {}'.format(name, pose))
                        continue
                    art = Art()
                    art.name = name
                    art.position = pose[0:3]
                    art.orientation = pose[3:7]
                    label = 0
                    for kwd in self.object_keywords:
                        if name.count(kwd) > 0:
                            art.label = label
                            break
                        label += 1
                    artefacts.append(art)
                    print('object "{}" found with position [{:.2f}, {:.2f}, {:.2f}] (XYZ) and orientation [{:.2f}, {:.2f}, {:.2f}] (RPY)'.format(name, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))
        return artefacts
    

    def pub(self, time):
        if self.is_initialized != True:
            pass

        # #{ PoseArray output
        
        posear_msg = PoseArray()
        posear_msg.header.frame_id = self.obj_frame
        posear_msg.header.stamp = rospy.Time.now()
        for art in self.objects:
            pose_msg = Pose()
            pose_msg.position.x = art.position[0]
            pose_msg.position.y = art.position[1]
            pose_msg.position.z = art.position[2]
            posear_msg.poses.append(pose_msg)
        self.pub_arts_gt.publish(posear_msg)
        
        # #} end of PoseArray output

        # #{ PointCloud output
        
        pcl_msg = PointCloud()
        pcl_msg.header.frame_id = self.obj_frame
        pcl_msg.header.stamp = rospy.Time.now()
        ch_lbls = ChannelFloat32()
        ch_lbls.name = "label"
        for art in self.objects:
            pt = Point32()
            pt.x = art.position[0]
            pt.y = art.position[1]
            pt.z = art.position[2]
            ch_lbls.values.append(art.label)
            pcl_msg.points.append(pt)
        pcl_msg.channels.append(ch_lbls)
        self.pub_pcl_arts_gt.publish(pcl_msg)
        
        # #} end of PointCloud output

        rospy.loginfo_throttle(1.0, 'Publishing a message of {} ground-truth object positions.'.format(len(posear_msg.poses)))


if __name__ == '__main__':
    rospy.init_node("artefact_gt_publisher")
    gt = ArtGtPub()
    rospy.loginfo('Spinning')
    rospy.spin()
    
