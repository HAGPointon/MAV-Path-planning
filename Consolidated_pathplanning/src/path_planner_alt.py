#!/usr/bin/env python

import roslib
import rospy
import tf
from tf.transformations import quaternion_from_euler as qfe
from actionlib import SimpleActionClient

import numpy as np
from math import radians

from geometry_msgs.msg import PolygonStamped, Point, PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path, Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

import shapely.geometry as geo

import shapely

# REMEMBER TO ADD THE MAIN THING THAT'S AT THE BOTTOM!!!

class PathPlannerNode(object):
    """
    This is a ROS node that is responsible for planning and executing
    the a path through the field.
    """
    def __init__(self):
        # Setup ROS node
        rospy.init_node('path_planner_alt')

        # Setup publishers and subscribers
        # Polygon subscriber
        rospy.Subscriber('heatmap_area', PolygonStamped, self.field_callback)

        # Robot position subscriber
        # To being with, this needs to be a stand in. ######
        rospy.Subscriber('odom', Odometry, self.odom_callback)

        # Marker publisher
        self.path_marker_pub = rospy.Publisher('visualization_marker',
                                               MarkerArray,
                                             latch=True)

        # Setup initial variables
        self.cut_spacing = 5
        # Field callback variables
        self.field_shape = None
        self.field_frame_id = None
        self.start_path_following = False
        # Odom callback variables
        self.robot_pose = None

        self.path = None
        self.path_status = None
        self.path_markers = None
        self.goal_state = None
        self.current_destination = None
        self.testing = True
        self.current_distance = None
        self.previous_destination = None
        self.clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        self.just_reset = False
        self.timeout = False

        # Spin until shutdown or we are ready for path following
        # This is the important bit!!!
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()
            # Start path following is set to true when polygon is received
            if self.start_path_following:
                # Run until stopped
                heading = 0
                # Setup path following
                # Calls the function setup_path_following #1
                self.setup_path_following(heading)
                # We are done so path following can close
                self.start_path_following = False


    def field_callback(self, msg):
        """
        Handles new field polygons, has to be called
        at least once before planning happens.
        """
        # Convert the PolygonStamped into a shapely polygon
        temp_points = []
        for point in msg.polygon.points:
            temp_points.append( (float(point.x), float(point.y)) )
        self.field_shape = geo.Polygon(temp_points)
        self.field_frame_id = msg.header.frame_id
        self.start_path_following = True

    def odom_callback(self, msg):
        """
        Watches for the robot's Odometry data, which is used in the path
        planning as the initial robot position.
        """
        self.robot_pose = msg


    #
    # 1
    #
    def setup_path_following(self, degrees=0):
        """
        Sets up the node for following the planned path.

        Will wait until the initial robot pose is set and until
        the move_base actionlib service is available.
        """
        # If testing prime the robot_pose
        if self.testing:
            self.robot_pose = Odometry()
            self.robot_pose.pose.pose.position.x = 10
            self.robot_pose.pose.pose.position.y = 10
        # Wait for the field shape
        # Subscriber setup above
        while self.field_shape == None:
            # Check to make sure ROS is ok still
            if rospy.is_shutdown(): return
            # Print message about the waiting
            msg = "Qualification: waiting on the field shape."
            rospy.loginfo(msg)
            rospy.Rate(1.0).sleep()
        # Wait for the robot position
        # Subscriber setup above
        while self.robot_pose == None:
            # Check to make sure ROS is ok still
            if rospy.is_shutdown(): return
            # Print message about the waiting
            msg = "Qualification: waiting on initial robot pose."
            rospy.loginfo(msg)
            rospy.Rate(1.0).sleep()
        # Now we should plan a path using the robot's initial pose
        origin = (self.robot_pose.pose.pose.position.x,
                  self.robot_pose.pose.pose.position.y)
        # Call function to plan path
        self.plan_path(self.field_shape, origin, degrees)
        rospy.loginfo("Path Planner: path planning complete.")
        return


    #
    # 2
    #
    def plan_path(self, field_polygon, origin=None, degrees=0):
        """
        This is called after a field polygon has been received.

        This uses the automow_planning.coverage module to plan a
        coverage path using the field geometry.  The path consists of
        a series of waypoints.
        """
        # Get the rotation to align with the longest edge of the polygon
        from automow_planning.maptools import rotation_tf_from_longest_edge, RotationTransform
        rotation = rotation_tf_from_longest_edge(field_polygon)
        rotation = RotationTransform(rotation.w + degrees)
        # Rotate the field polygon
        from automow_planning.maptools import rotate_polygon_to
        transformed_field_polygon = rotate_polygon_to(field_polygon, rotation)
        # Decompose the rotated field into a series of waypoints
        from automow_planning.coverage import decompose
        print origin
        if origin is not None:
            point_mat = np.mat([[origin[0], origin[1], 0]], dtype='float64').transpose()
            origin = rotation.irm * point_mat
            origin = (origin[0,0], origin[1,0])
            #
            # Key point here!!
            #
            # Decompose calls Generate_intersections returns lines,
            #
            # Decompose then calls order_points returns a list of points
            #
        transformed_path = decompose(transformed_field_polygon,
                                     origin=(origin[0], origin[1]),
                                     width=self.cut_spacing)
        # Rotate the transformed path back into the source frame
        from automow_planning.maptools import rotate_from
        self.path = rotate_from(np.array(transformed_path), rotation)
        # Calculate headings and extend the waypoints with them # 3
        self.path = self.calculate_headings(self.path)
        # Set the path_status to 'not_visited'
        self.path_status = []
        for waypoint in self.path:
            self.path_status.append('not_visited')
        # Visualize the data
        # visualize_path not done, just passes to another thing
        # self.visualize_path(self.path, self.path_status) # 4
        self.visualize_path_as_marker(self.path, self.path_status)


    #
    # 3
    #
    def calculate_headings(self, path):
        """
        Calculates the headings between paths and adds them to the waypoints.
        """
        new_path = []
        for index, waypoint in enumerate(path):
            new_path.append(list(path[index]))
            # If the end, copy the previous heading
            if index == 0:
                new_path[index].append(0)
                continue
            # Calculate the angle between this waypoint and the next
            dx = path[index][0] - path[index-1][0]
            dy = path[index][1] - path[index-1][1]
            from math import atan2, pi
            heading = atan2(dy, dx)
            new_path[index].append(heading)
        return new_path


    #
    # 4
    #
    def visualize_path_as_marker(self, path, path_status):
        """
        Publishes visualization Markers to represent the planned path.

        Publishes the path as a series of spheres connected by lines.
        The color of the spheres is set by the path_status parameter,
        which is a list of strings of which the possible values are in
        ['not_visited', 'visiting', 'visited'].
        """
        # Get the time
        now = rospy.Time.now()
        # If self.path_markers is None, initialize it
        if self.path_markers == None:
            self.path_markers = MarkerArray()
        # # If there are existing markers, delete them
        # markers_to_delete = MarkerArray()
        # if len(self.path_markers.markers) > 0:
        #     for marker in self.path_markers.markers:
        #         marker.action = Marker.DELETE
        #         markers_to_delete.markers.append(marker)
        #     self.path_marker_pub.publish(markers_to_delete)
        # Clear the path_markers
        self.path_markers = MarkerArray()
        line_strip_points = []
        # Create the waypoint markers
        for index, waypoint in enumerate(path):
            waypoint_marker = Marker()
            waypoint_marker.header.stamp = now
            waypoint_marker.header.frame_id = self.field_frame_id
            waypoint_marker.ns = "waypoints"
            waypoint_marker.id = index
            waypoint_marker.type = Marker.ARROW
            if index == 0:
                waypoint_marker.type = Marker.CUBE
            waypoint_marker.action = Marker.MODIFY
            waypoint_marker.scale.x = 2
            waypoint_marker.scale.y = 1
            waypoint_marker.scale.z = 1
            point = Point(waypoint[0], waypoint[1], 0)
            waypoint_marker.pose.position = point
            # Store the point for the line_strip marker
            line_strip_points.append(point)
            # Set the heading of the ARROW
            quat = qfe(0, 0, waypoint[2])
            waypoint_marker.pose.orientation.x = quat[0]
            waypoint_marker.pose.orientation.y = quat[1]
            waypoint_marker.pose.orientation.z = quat[2]
            waypoint_marker.pose.orientation.w = quat[3]
            # Color is based on path_status
            status = path_status[index]
            if status == 'not_visited':
                waypoint_marker.color = ColorRGBA(1,0,0,0.5)
            elif status == 'visiting':
                waypoint_marker.color = ColorRGBA(0,1,0,0.5)
            elif status == 'visited':
                waypoint_marker.color = ColorRGBA(0,0,1,0.5)
            else:
                rospy.err("Invalid path status.")
                waypoint_marker.color = ColorRGBA(1,1,1,0.5)
            # Put this waypoint Marker in the MarkerArray
            self.path_markers.markers.append(waypoint_marker)
        # Create the line_strip Marker which connects the waypoints
        line_strip = Marker()
        line_strip.header.stamp = now
        line_strip.header.frame_id = self.field_frame_id
        line_strip.ns = "lines"
        line_strip.id = 0
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = Marker.ADD
        line_strip.scale.x = 0.1
        line_strip.color = ColorRGBA(0,0,1,0.5)
        line_strip.points = line_strip_points
        self.path_markers.markers.append(line_strip)
        # Publish the marker array
        self.path_marker_pub.publish(self.path_markers)


if __name__ == '__main__':
    ppn = PathPlannerNode()