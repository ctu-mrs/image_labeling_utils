#!/usr/bin/python

import os
import re
import rospy
from geometry_msgs.msg import PoseStamped
# from image_labeling_utils import artefact_gtConfig


class ArtGtPub():
    def __init__(self):

        self.is_initialized = False
        self.art_gt_pub = rospy.Publisher("artefact_gt", PoseStamped, queue_size=10)

        self.sdf_path =  rospy.get_param("~sdf_path","")
        self.object_name =  rospy.get_param("~object_name","")
        self.obj_frame =  rospy.get_param("~obj_frame","")


        if self.sdf_path == "" or self.object_name == "":
            rospy.logerr('No sdf file specified, or object is not specified, abort')
            rospy.signal_shutdown("Sdf file or object  is not specified")

        if self.obj_frame == "":
            rospy.logerr('Frame is not specified, abot')
            rospy.signal_shutdown("")


        self.keywords = [
            'rescue',
            'phone',
            'backpack',
            'rope',
            'helmet'
        ]

        self.objects = self.parse_lines(self.sdf_path)
        self.object = self.get_object(self.object_name)
        if self.object is False:
            rospy.logerr('Object is not found, abort')
            rospy.signal_shutdown("Object is not found")


        rospy.loginfo('Node is initialized')
        self.is_initialized = True

        
        pub_timer  = rospy.Timer(rospy.Duration(1.0/10.0), self.pub)

        

    def read_file(self,path):
        f = open(path, 'r')
        return f.readlines()

    def parse_lines(self,path):

        lines = self.read_file(path)
        artefacts =  []
        for i in range(len(lines)):
            if lines[i].count("include"):
                if any(x in lines[i+1] for x in self.keywords):
                    name = re.findall('\>(.*?)\<',lines[i+1])[0]
                    print('artefact: {}'.format(name))
                    pose = re.findall('\>(.*?)\<',lines[i+3])[0].split(' ')
                    print('pose is x: {} y: {} z: {}'.format(pose[0], pose[1], pose[2]))
                    artefacts.append((name, pose[0], pose[1], pose[2]))
                name = lines[i]
        return artefacts
    
    def get_object(self, name):
        if len(self.objects) == 0:
            rospy.signal_shutdown('empty object list')
        for obj in self.objects:
            if self.object_name == obj[0]:
                rospy.loginfo('object {} is found pose is {}'.format(obj[0], (obj[1], obj[1], obj[2])))
                return obj
        return False


    def pub(self, time):
        if self.is_initialized != True:
            pass

        obj_msg = PoseStamped()
        obj_msg.header.frame_id = self.obj_frame
        obj_msg.pose.position.x = float(self.object[1])
        obj_msg.pose.position.y = float(self.object[2])
        obj_msg.pose.position.z = float(self.object[3])

        self.art_gt_pub.publish(obj_msg)



if __name__ == '__main__':
    rospy.init_node("your_sensor_node")
    gt = ArtGtPub()
    rospy.spin()
    




