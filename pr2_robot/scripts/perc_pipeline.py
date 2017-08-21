#!/usr/bin/env python

"""
NOTE: Need the sensor_stick dir from the Perception Exercise 3, to be in the src directory of the 
catkin_ws along with RoboND-Perception-Project (see under Exercise 3 code below for more details).
"""

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

# Add Exercise-2 Code (from segmentation.py in Exercise 2) marked by #####:

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

    ##### RANSAC plane segmentation #####

    """RANSAC (Random Sample Consensus) is a two step (hypothesis and verification) iterative method
    which identifies data points belonging to a mathematical model (inliners) and those that dont (outliners)."""

    """First, the model is constructed using a min. no. of data pts. (eg. two for a line) and then the rest of
    pts. are verfied against its parameters (eg. slope and y-cutoff for a line) with certain error thresholds.
    The set of inliers obtained for that fitting model (random sample) is called a consensus set.
    The two steps are repeated until the obtained consensus set in certain iteration has enough inliers
    and that sample (mathematical model parameters) forms the solution as it had the most inliners in consensus."""

    # The points chosen are random so the solution is probalistic, increasing with the number of iterations.

    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit.
    # RANSAC plane fitting algorithm (calculate plane parameters and verfiy) already exists in the PCL library.
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model.
    # This is the error threshold for the model fit and influences (increases) the consensus set.
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inliner indices and model coefficients
    inliers, coefficients = seg.segment()

    ##### Extract inliers and outliers #####

    # Extract inliers
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    cloud_table = extracted_inliers

    # Extract outliers using the negative flag to True.
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)
    cloud_objects = extracted_outliers

    ##### Euclidean Clustering #####

    """Euclidean Clustering is the DBSCAN algorithm as it uses the Euclidean Distance to identfy nearest neighbours, 
    if the distance b/w is < min. distance specified, then point is added to the cluster (inliners), else outliner.
    If the point has > (min. members of a cluster - 1) neigbours, it becomes a core member, else an edge member.
    Each point that can be in a cluster is identified and then the algorithm moves to the next random point."""

    """Using k-d trees for nearest neighbor search for PCL's Euclidian Clustering (DBSCAN)
    algorithm to decrease the computational burden.
    k-d trees segment the Euclidian Space into partitions by divinding each dimension sequentially (at each root)
    into two each time (forming a tree) using the median for each dimension, same as in the Quick Sort partion method.
    Each point is then located in a partition and the seach is focussed there instead of the whole space."""
    
    """Convert XYZRGB point cloud to XYZ with helper function from pcl_helper, because PCL's 
    Euclidean Clustering algorithm requires a point cloud with only spatial information."""
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    ##### Create Cluster-Mask Point Cloud to visualize each cluster separately. #####

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold (max. Euclidean Distance b/w points)
    # as well as minimum and maximum cluster size (in points).
    # Experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(2000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Assign a color corresponding to each segmented object in scene.
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color.
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    ##### Convert PCL data to ROS messages #####

    """Convert PCL data (PointXYZRGB format) to ROS msg (type PointCloud2)
    with helper function from pcl_helper."""
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    ##### Publish ROS messages #####
    #This is just for testing so we publish the whole input itself.
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 Code (from capture_features.py and features.py) marked by #####:
    """
    NOTE: Copy the sensor_stick directory from the Perception Ex 3 to the src in catkin_ws along with the 
    RoboND-Perception-Project directory. Then edit the model list in capture_features.py in sensor_stick 
    to match the models of the environment as in pick_list_(env#).yaml files in /pr2_robot/config/ and 
    generate the features and train the SVM to those models as done in Ex 3.
    """

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        ##### Convert the cluster from pcl to ROS using helper function. #####
        ros_cluster = pcl_to_ros(pcl_cluster)

        ##### Extract histogram features as in capture_features.py #####

        """The functions compute_color_histograms() and compute_normal_histograms() 
        are from features.py and are explained there. The rest are in capture_features.py, 
        both of which are in sensor_stick directory from Exercise 3 which needs to be 
        copied to the same src directory in catkin_ws as the RoboND-Perception-Project."""

        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)

        ##### Compute the associated feature vector #####
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output needed to complete the project.
    detected_objects_pub.publish(detected_objects)

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

    # Creating a publisher to publish the point cloud data for the cluster cloud to topic called pcl_cluster.
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    ##### Create New Publishers for detected objects #####

    """Creating two new publishers, object_markers_pub and detected_objects_pub
    that publish to topics "/object_markers" and "/detected_objects" with 
    Message Types "Marker" and "DetectedObjectsArray", respectively."""
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    ##### Initialize color_list #####
    # This is needed to initalize the color_list attribute of the helper function get_color_list()
    get_color_list.color_list = []

    ##### Spin while node is not shutdown #####
    while not rospy.is_shutdown():
     rospy.spin()
