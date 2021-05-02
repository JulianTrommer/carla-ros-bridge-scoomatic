#!/usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import rospy
import tf

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2



# ==============================================================================
# -- OdometryPublisher---------------------------------------------------------
# ==============================================================================


class OdometryPublisher(object):

    def __init__(self):
        self.loop_rate = rospy.Rate(10)
        self.role_name = rospy.get_param("~role_name", 'ego_vehicle')

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.got_init_pos = 0
        self.init_pos_x = 0.0
        self.init_pos_y = 0.0
        self.init_pos_z = 0.0
        self.init_pos_roll = 0.0
        self.init_pos_pitch = 0.0
        self.init_pos_yaw = 0.0

        self.start = rospy.Time.now()
        self.start_sec = self.start.to_sec()
        self.stop = self.start_sec + 10
        self.counter = 0

        self.trans_init = [0.0, 0.0, 0.0]
        self.rot_init = [0.0, 0.0, 0.0, 1.0]
        
        self.trans_map_egovehicle = [0.0, 0.0, 0.0]
        self.rot_map_egovehicle = [0.0, 0.0, 0.0, 1.0]
        self.trans_odom_egovehicle = [0.0, 0.0, 0.0]
        self.rot_odom_egovehicle = [0.0, 0.0, 0.0, 1.0]

        # ==========================================
        # -- Subscriber ----------------------------
        # ==========================================

        # position data from Localization
        self.odom_data = Odometry()
        self.odom_data_subscriber = rospy.Subscriber(
            "/carla/{}/odometry".format(self.role_name), Odometry, self.odom_data_updated)
        self.tflistener = tf.TransformListener()

        self.laser_scan_subscriber = rospy.Subscriber(
            "/carla/{}/laserscan".format(self.role_name), LaserScan, self.laser_scan_updated)
        #self.laser_scan_subscriber = rospy.Subscriber(
        #    "scan".format(self.role_name), LaserScan, self.laser_scan_updated)

        self.pointcloud_subscriber = rospy.Subscriber(
            "/carla/{}/lidar/lidar1/point_cloud".format(self.role_name), PointCloud2, self.pointcloud_updated)

        # ==========================================
        # -- Publisher ----------------------------
        # ==========================================

        # position data from Localization
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.laser_scan_pub = rospy.Publisher("base_scan_laser", LaserScan, queue_size=50)

        self.pointcloud_pub = rospy.Publisher("base_scan_lidar", PointCloud2, queue_size=50)


    def pointcloud_updated(self, pointcloud):
        """
        Callback on position updates
        """
        
        self.pointcloud = pointcloud
        self.pointcloud.header.frame_id = "lidar_base_link"
        self.pointcloud_pub.publish(self.pointcloud)

    def laser_scan_updated(self, laser_scan):
        """
        Callback on position updates
        """
        
        self.laser_scan = laser_scan
        self.laser_scan.header.frame_id = "lidar_base_link"
        self.laser_scan_pub.publish(self.laser_scan)


    def odom_data_updated(self, odom_data):
        """
        Callback on position updates
        Publishes odometry msg (map -> ego_vehicle transformed to odom_base_link -> base_link)
        Attention: Only right Message if odom_base_link has same position as map frame
        """
        
        #self.odom_data = odom_data
        #self.twist = self.odom_data.twist.twist
        #(trans,rot) = self.tflistener.lookupTransform('odom_origin', '/ego_vehicle', rospy.Time(0))

        odom = odom_data
        #odom.header.stamp = self.current_time
        odom.header.frame_id = "odom_base_link"

        # set the position
        #odom.pose.pose = Pose(Point(trans[0], trans[1], trans[2]), Quaternion(*rot))

        # set the velocity
        odom.child_frame_id = "base_link"
        #odom.twist.twist = self.twist

        # publish the message
        self.odom_pub.publish(odom)

    def publish_odom(self, publish_odom_in_ego_vehicle_frame, publish_map_odom_tf):
        if publish_odom_in_ego_vehicle_frame:
            self.odom_broadcaster.sendTransform((self.trans_init[0], self.trans_init[1], self.trans_init[2]), self.rot_init, self.current_time, "odom_origin", "map")
        else:
            self.odom_broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), self.current_time, "odom_origin", "map")
        if publish_map_odom_tf:
            (self.trans_odom, self.rot_odom) = self.tflistener.lookupTransform('map', 'odom_origin', rospy.Time(0))
            self.odom_broadcaster.sendTransform((self.trans_odom[0], self.trans_odom[1], self.trans_odom[2]), self.rot_odom, self.current_time, "odom_base_link", "map")
        (self.trans_odom_egovehicle, self.rot_odom_egovehicle) = self.tflistener.lookupTransform('odom_origin', 'ego_vehicle', rospy.Time(0))
        self.odom_broadcaster.sendTransform((self.trans_odom_egovehicle[0], self.trans_odom_egovehicle[1], self.trans_odom_egovehicle[2]), self.rot_odom_egovehicle, self.current_time, "base_link", "odom_base_link")
        #self.odom_broadcaster.sendTransform((self.trans_odom_egovehicle[0], self.trans_odom_egovehicle[1], self.trans_init[2]), self.rot_odom_egovehicle, self.current_time, "base_link", "odom_base_link")
        self.odom_broadcaster.sendTransform((0.0, 0.0, 1.2), (0.0, 0.0, 0.0, 1.0), self.current_time, "lidar_base_link", "base_link")

    #def publish_odom_in_map_frame(self):
        #self.odom_broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), self.current_time, "odom_base_link", "map")
        #(self.trans_odom_egovehicle, self.rot_odom_egovehicle) = self.tflistener.lookupTransform('odom_base_link', 'ego_vehicle', rospy.Time(0))
        #self.odom_broadcaster.sendTransform((self.trans_odom_egovehicle[0], self.trans_odom_egovehicle[1], self.trans_odom_egovehicle[2]), self.rot_odom_egovehicle, self.current_time, "base_link", "odom_base_link_gmap")

        #self.odom_broadcaster.sendTransform((0.0, 0.0, 2.4), (0.0, 0.0, 0.0, 1.0), self.current_time, "lidar_base_link", "base_link")

    def __del__(self):
        """
        Unregister all publisher and subscriber
        """
        rospy.loginfo("Unregister topics...")
        self.odom_data_subscriber.unregister()

    def run(self):
        """
        main loop
        """
        # wait for ros-bridge to set up CARLA world
        rospy.loginfo("OdometryPublisher is running!")

        publish_odom_in_ego_vehicle_frame = rospy.get_param('publish_odom_in_ego_vehicle_frame', False)
        publish_map_odom_tf = rospy.get_param('publish_map_odom_tf', True)

        #self.x = 0.0
        #self.y = 0.0
        #self.z = 0.0
        #self.roll = 0
        #self.pitch = 0
        #self.yaw = 0
        #self.odom_quat = Quaternion()
        #trans_init = [0.0, 0.0, 0.0]
        #rot_init = [0.0, 0.0, 0.0, 1.0]
        got_init_pos = 0
        #trans_map_egovehicle = [0.0, 0.0, 0.0]
        #rot_map_egovehicle = [0.0, 0.0, 0.0, 1.0]
        #trans_odom_egovehicle = [0.0, 0.0, 0.0]
        #rot_odom_egovehicle = [0.0, 0.0, 0.0, 1.0]

        #self.vx = 0.0
        #self.vy = 0.0
        #self.vth = 0.0

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.current_time = rospy.Time.now()
                if got_init_pos < 50:
                    try:
                        (trans_map_egovehicle, rot_map_egovehicle) = self.tflistener.lookupTransform('map', 'ego_vehicle', rospy.Time(0))
                        got_init_pos = got_init_pos + 1
                    except (tf.LookupException, tf.ExtrapolationException):
                        #print("Exception")
                        #rospy.logwarn("Exception im Odometry Publisher. It's normal if this Exception occurrs directly after starting the Odometry Publisher. It's critical if it occurrs not at startup")
                        continue
                    if got_init_pos == 50:
                        self.trans_init = trans_map_egovehicle
                        self.rot_init = rot_map_egovehicle

                try:
                    self.publish_odom(publish_odom_in_ego_vehicle_frame, publish_map_odom_tf)
                except (tf.LookupException, tf.ExtrapolationException):
                    rospy.logwarn("Exception im Odometry Publisher. It's normal if this Exception occurrs directly after starting the Odometry Publisher. It's critical if it occurrs not at startup")
                    continue

                #self.odom_broadcaster.sendTransform((trans_init[0], trans_init[1], trans_init[2]), rot_init, self.current_time, "odom_base_link", "map")
                #try:
                #    (trans_odom_egovehicle,rot_odom_egovehicle) = self.tflistener.lookupTransform('odom_base_link', 'ego_vehicle', rospy.Time(0))
                #except (tf.LookupException, tf.ExtrapolationException):
                #    continue

                #self.odom_broadcaster.sendTransform((trans_odom_egovehicle[0], trans_odom_egovehicle[1], trans_odom_egovehicle[2]), rot_odom_egovehicle, self.current_time, "base_link", "odom_base_link_gmap")

                #self.odom_broadcaster.sendTransform((0.0, 0.0, 2.4), (0.0, 0.0, 0.0, 1.0), self.current_time, "lidar_base_link", "base_link")

                self.last_time = self.current_time
                self.r.sleep()
            except rospy.ROSInterruptException:
                pass


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    rospy.init_node("odometry_publisher", anonymous=True)

    odometry_publisher = None

    try:
        odometry_publisher = OdometryPublisher()
        odometry_publisher.run()
    finally:
        if odometry_publisher is not None:
            del odometry_publisher
        rospy.loginfo("Done")



if __name__ == '__main__':
    main()
