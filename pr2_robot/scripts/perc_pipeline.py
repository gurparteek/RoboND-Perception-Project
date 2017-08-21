#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Add Exercise-2 Code:

    ##### Convert ROS msg to PCL data #####

    """Convert ROS msg (type PointCloud2) to PCL data (PointXYZRGB format)
    with helper function from pcl_helper."""
    cloud = ros_to_pcl(pcl_msg)
    
    ##### Statistical Outlier Filtering #####

    # Creating a filter object.
    outlier_filter = cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(10)

    # Set threshold scale factor
    x = 0.001

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()

    ##### Voxel Grid Downsampling #####

    """The point clouds from RGB-D cameras are too dense, hence computationally expensive. Downsampling 
    the point cloud data to reduce density but preserve important information is ideal.

    Using a Voxel Grid Filter where a grid of volumetric elements (voxels; as pixel is to picture element)
    is made and each voxel is averaged to a point cloud element; downsampled."""

    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud_filtered.make_voxel_grid_filter()

    """Choose a voxel (also known as leaf) size (units in meters).
    Should start small and keep going large till loss of important information starts."""

    """A good way to choose leaf size is knowing the important information data forehand 
    such as smallest (or target) object size."""
    LEAF_SIZE = 0.01
    """A voxel (leaf) size of 0.01 results in a voxel of 1e-6 cubic meters that retains
    most of the important information, while significantly reducing the number of points in the cloud."""  

    # Set the voxel (or leaf) size. 
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud.
    cloud_filtered = vox.filter()

    ##### PassThrough filter #####

    """More points in cloud = more coumputation; so if the target object location is known,
    the rest of the point cloud is not needed."""

    """A pass through filter is like a cropping tool. We specify an axis along which we know the limits
    within which the target objects lie, known as the region of interest. The pass through filter passes 
    through the cloud leaving only the region of interest."""

    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    # Applying the filter along z axis (the height with respect to the ground) to our tabletop scene.
    filter_axis = 'z'
    passthrough.set_filter_field_name (filter_axis)
    # Setting axis values to keep only the objects and the flat table top.
    axis_min = 0.605
    axis_max = 0.72
    # The axis min and max sets the region of interest that the filter leaves out as a window as it passes.
    passthrough.set_filter_limits (axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation

    # TODO: Extract inliers and outliers

    # TODO: Euclidean Clustering

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    ##### Convert PCL data to ROS messages #####

    """Convert PCL data (PointXYZRGB format) to ROS msg (type PointCloud2)
    with helper function from pcl_helper."""
    ros_cloud_objects = pcl_to_ros(cloud_filtered)
    ros_cloud_table = pcl_to_ros(cloud_filtered)

    ##### Publish ROS messages #####
    #This is just for testing so we publish the whole input itself.
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)

        # Grab the points for the cluster

        # Compute the associated feature vector

        # Make the prediction

        # Publish a label into RViz

        # Add the detected object to the list of detected objects.

    # Publish the list of detected objects

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables

    # TODO: Get/Read parameters

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list

        # TODO: Get the PointCloud for a given object and obtain it's centroid

        # TODO: Create 'place_pose' for the object

        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file



if __name__ == '__main__':

    ##### ROS node initialization #####

    # Initializing a new node.
    rospy.init_node('object_reco', anonymous=True)

    ##### Create Subscribers #####

    """Subscribing our node to the "/pr2/world/points" topic so that anytime a message arrives,
    the message data (a point cloud) will be passed to the pcl_callback() function for processing."""
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    ##### Create Publishers #####

    # Creating two publishers to publish the point cloud data for the table and the objects to topics
    # called pcl_table and pcl_objects, respectively.
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)

    ##### Spin while node is not shutdown #####
    while not rospy.is_shutdown():
     rospy.spin()
