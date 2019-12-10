#!/usr/bin/env python
# import required packages
import rospy, math, os
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist , Pose2D
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import rospkg
import yaml

class Tags():
       
    def __init__(self):
        self.tag_target=PoseStamped()
        self.X=None
        self.Y=None
        self.TZ=None
        self.iter=0
        self._xtot=0
        self._ytot=0
        self._tagposetot=0
        rospy.init_node("AR_localization")

        #Publish on /AR_target_pose ,topic as a PoseStamped message
        self.tag_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1 )

        #Subscribe pose from amcl_pose
        rospy.Subscriber("amcl", PoseWithCovarianceStamped, self.get_turtlePose)

        #subscribe pose from alvar markers
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.get_tags)
        rospy.loginfo("Publishing marker pose for particular tag id")
        rospy.spin()

    def import_yaml(self):
        rospack = rospkg.RosPack()
        with open(rospack.get_path('ar_tag_detection')+"/config/tags.yaml", 'r') as stream:
            try:
                tag_locations=yaml.safe_load(stream)
                #print(bay_lib)
            except yaml.YAMLError as exc:
                print(exc)
                print("check Global_planner.yaml in config folder")
            return tag_locations
    
    def get_turtlePose(self, msg):
        # get the pose of turtlebot
        self.X = msg.pose.pose.position.x
        self.Y = msg.pose.pose.position.y
        turtleZrot = msg.pose.pose.orientation.z
        turtleWrot = msg.pose.pose.orientation.w
        self.TZ = math.atan2(2*turtleZrot*turtleWrot, 1-2*turtleZrot*turtleZrot)
 
    def get_tags(self, data):
        try:
            
            # msg_pose=PoseWithCovarianceStamped()
            xtot=0
            ytot=0
            tagposetot=0
            for i in range(len(data.markers)):
                msg=data.markers[i]
                id_tag=msg.id
                AR_x = msg.pose.pose.position.x
                AR_y = msg.pose.pose.position.y
                AR_Z = msg.pose.pose.position.z
                Xrot = msg.pose.pose.orientation.y
                Yrot = msg.pose.pose.orientation.x
                Zrot = msg.pose.pose.orientation.z
                Wrot = msg.pose.pose.orientation.w
                # AR_tz = math.atan2(2*Zrot*Wrot, 1-2*Zrot*Zrot)
                siny_cosp = 2 * (Wrot * Zrot + Xrot * Yrot)
                cosy_cosp = 1 - 2 * (Yrot * Yrot + Zrot * Zrot)
                AR_tz = math.atan2(siny_cosp, cosy_cosp)



                tag_location=self.import_yaml()

                Tag_pose=tag_location['AR_tags'][str(id_tag)]

                camera_to_tag=np.array([[math.cos(AR_tz),-math.sin(AR_tz),0,AR_x],[math.sin(AR_tz),math.cos(AR_tz),0,AR_y],[0,0,1,AR_Z],[0,0,0,1]])
                tag_to_camera=np.linalg.inv(camera_to_tag)
                # tag_to_camera=(camera_to_tag)
                map_to_tag=np.array([[math.cos(Tag_pose[5]),-math.sin(Tag_pose[5]),0,Tag_pose[0]],[math.sin(Tag_pose[5]),math.cos(Tag_pose[5]),0,Tag_pose[1]],[0,0,1,Tag_pose[2]],[0,0,0,1]])

                map_to_camera=np.dot(map_to_tag,tag_to_camera)
                #mean of poses    
                xtot+=map_to_camera[1,3]
                ytot+=map_to_camera[0,3]
                if(abs(Tag_pose[5])==1.57):
                    Tag_pose[5]=-Tag_pose[5]
                # else:
                    # Tag_pose[5]=Tag_pose[5]
                tagposetot+=Tag_pose[5]

            self.iter+=1
            map_to_camera[1,3]=xtot/(len(data.markers)) 
            map_to_camera[0,3]=ytot/(len(data.markers)) 
            Tag_pose[5]=tagposetot/(len(data.markers))
            self._xtot+=map_to_camera[1,3]
            self._ytot+=map_to_camera[0,3]
            self._tagposetot+=Tag_pose[5]
            
            print("iter",self.iter)
            if self.iter>=10:
                map_to_camera[1,3]=self._xtot/self.iter
                map_to_camera[0,3]=self._ytot/self.iter
                Tag_pose[5]=self._tagposetot/self.iter
                
                msg_pose=PoseWithCovarianceStamped()
                msg_pose.pose.pose.position.x=map_to_camera[1,3]
                msg_pose.pose.pose.position.y=map_to_camera[0,3]
                # if(abs(Tag_pose[5])==1.57):
                #     msg_pose.pose.pose.orientation.z=-Tag_pose[5]
                # else:
                #     msg_pose.pose.pose.orientation.z=Tag_pose[5]
                msg_pose.pose.pose.orientation.z=Tag_pose[5]
                
                # msg_pose.pose.pose.orientation.z=math.atan(map_to_camera[1,0]/map_to_camera[0,0])
                msg_pose.pose.pose.orientation.w=1
                # rate = rospy.Rate(1000)
                # rate.sleep()
                self.tag_pub.publish(msg_pose)
                # set every thing to zero
                self._xtot=0
                self._ytot=0
                self._tagposetot=0
                self.iter=0

                
        except:
            rospy.loginfo("Tag is not seen")
            self._xtot=0
            self._ytot=0
            self._tagposetot=0
            self.iter=0

        '''

        #publish target0 flag on target0_Flag
        target0_flag = rospy.Publisher("target0_Flag", Bool, queue_size=10)
        rospy.loginfo("publishing target0 flag set on /taget0_Flag topic")
        i = 1
        while i < 50:
            i += 1
            target0_flag.publish(self.t0_flag)


        rate = rospy.Rate(10)
        #publish target0 flag on target0_Flag
        target1_flag = rospy.Publisher("target1_Flag", Bool, queue_size=10)
        rospy.loginfo("publishing target1 flag set on /taget1_Flag topic")
        i = 1
        while i < 50:
            i += 1
            target1_flag.publish(self.t1_flag)"""

        if self.t0_flag == True and self.t1_flag == True:
            goToOn = rospy.Publisher("goToOn_flag", Bool, queue_size=10)
            goToOn_flag = True
            i = 1
            while i < 100:
                i += 1
                goToOn.publish(goToOn_flag)
            return
        else:
            goToOn = rospy.Publisher("goToOn_flag", Bool, queue_size=10)
            goToOn_flag = False
            goToOn.publish(goToOn_flag)
            

        #get number of amrkers detected
        n = len(msg.markers)

        if n == 0:
            return

        #for each markers (0,1) detected do the following
        for tag in msg.markers:
            if tag.id == 0:
                #counter if required
                if self.t0_flag == True:
                    continue

                affine_transf = [[math.cos(turtleAngle), -math.sin(turtleAngle), 0, turtleX], [math.sin(turtleAngle), math.cos(turtleAngle), 0, turtleY], [0, 0, 1, turtleZ], [0, 0, 0, 1]]
                """affine_transf = numpy.array([[1-2*turtleZrot*turtleZrot, 2*turtleZrot*turtleWrot, 0, turtleX], [-2*turtleZrot*turtleWrot, 1-2*turtleZrot*turtleZrot, 0, turtleY], [0, 0, 1, turtleZ], [0, 0, 0, 1]])"""
                point_in_baseframe = [[tag.pose.pose.position.x], [tag.pose.pose.position.y], [tag.pose.pose.position.z], [1]]
                point_in_mapframe = numpy.matmul(affine_transf, point_in_baseframe)

                self.xx0 = self.xx0 + point_in_mapframe[0]
                self.yy0 = self.yy0 + point_in_mapframe[1]
                self.zz0 = self.zz0 + point_in_mapframe[2] 
                self.counter0 += 1

                if self.counter0 == 1:
                    tag_target0.pose.position.x = self.xx0/1.0
                    tag_target0.pose.position.y = self.yy0/1.0               
                    tag_target0.pose.position.z = self.zz0/1.0
                   
                    tag_target0.pose.orientation.w = 1

                    #add time stamp and frameid
                    tag_target0.header.stamp = rospy.Time.now()
                    tag_target0.header.frame_id = "map"
                   
                    self.t0_flag = True



            elif tag.id == 1:
                #counter if required
                if self.t1_flag == True:
                    continue

                affine_transf = numpy.array([[math.cos(turtleAngle), -math.sin(turtleAngle), 0, turtleX], [math.sin(turtleAngle), math.cos(turtleAngle), 0, turtleY], [0, 0, 1, turtleZ], [0, 0, 0, 1]])
                """affine_transf = numpy.array([[1-2*turtleZrot*turtleZrot, 2*turtleZrot*turtleWrot, 0, turtleX], [-2*turtleZrot*turtleWrot, 1-2*turtleZrot*turtleZrot, 0, turtleY], [0, 0, 1, turtleZ], [0, 0, 0, 1]])"""
                point_in_baseframe = numpy.array([[tag.pose.pose.position.x], [tag.pose.pose.position.y], [tag.pose.pose.position.z], [1]])
                point_in_mapframe = numpy.matmul(affine_transf, point_in_baseframe)

                self.xx1 = self.xx1 + point_in_mapframe[0]
                self.yy1 = self.yy1 + point_in_mapframe[1]
                self.zz1 = self.zz1 + point_in_mapframe[2] 
                self.counter1 += 1

                if self.counter1 == 5:
                    tag_target1.pose.position.x = self.xx1/5.0
                    tag_target1.pose.position.y = self.yy1/5.0               
                    tag_target1.pose.position.z = self.zz1/5.0
                   
                    tag_target1.pose.orientation.w = 1

                    #add time stamp and frameid
                    tag_target1.header.stamp = rospy.Time.now()
                    tag_target1.header.frame_id = "map"
                   
                    self.t1_flag = True
            else:
                continue
        '''
if __name__ == '__main__':
    try:
        Tags()
        # rospy.spin()
        # rospy.sleep(0.01)
    except rospy.ROSInterruptException:
        rospy.loginfo("Localization node terminated.")