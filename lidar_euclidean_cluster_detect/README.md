# lidar_euclidean_cluster_detect

The purpose of this package is to detect individual objects in pointcloud data.
Points are grouped into clusters based on proximity and published as detected objects.

NOTE: A new version of this package is available in autoware.auto.

## Process

1. Pointcloud preprocessing
	- Points closer than a distance of `remove_points_upto` meters are removed from the cloud.
	- Points are then downsampled if the `downsample_cloud` parameter is set to true.
	- The pointcloud is trimmed to remove points based on height thresholds (`clip_min_height` and `clip_max_height`).
	- Points are further trimmed based on their y position to either side of the vehicle if `keep_lanes` is set to true. The bounds are defined by `keep_lane_left_distance` and `keep_lane_right_distance`.
	- A RANSAC-based algorithm is then used to determine a ground plane and remove any points belonging to the ground.
	This is activated by the `remove_ground` parameter.
	- The pointcloud is further filtered using Difference-of-Normals to remove any points that belong to a smooth surface.
	This is activated by the `use_diffnormals` parameter.

2. Pointcloud Clustering
	- The preprocessed pointcloud is then clustered using Euclidean Cluster Extraction, the cluster tolerance is defined by the `clustering_distance` parameter.
	This is the only part of the node that provides the option to use the GPU (activated by the `use_gpu` parameter).
	- Resulting clusters are then checked against neighboring clusters and any clusters which are less than `cluster_merge_threshold` apart are combined into a single cluster.
	- Rectangluar bounding boxes and polygonal bounds are then fit to the cluster pointclouds.

#### References

[Voxel-based Downsampling](http://pointclouds.org/documentation/tutorials/voxel_grid.php)  
[Pointcloud Surface Normal Estimation](http://pointclouds.org/documentation/tutorials/normal_estimation.php)  
[Difference of Normals Segmentation](http://pointclouds.org/documentation/tutorials/don_segmentation.php)  
[Euclidean Cluster Extraction](http://pointclouds.org/documentation/tutorials/cluster_extraction.php)  

## ROS API

#### Subs
- `points_raw` ([sensor_msgs/PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html))  
Input pointcloud from lidar sensor.

#### Pubs
- `detection/lidar_detector/cloud_clusters` (autoware_msgs/CloudClusterArray)  
Array of cloud clusters.
- `detection/lidar_detector/objects` (autoware_msgs/DetectedObjectArray)  
Array of all detected objects.
- `cluster_centroids` (autoware_msgs/Centroids)  
Centroids of the clusters.
- `points_lanes` ([sensor_msgs/PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html))  
Pointcloud with all preprocessing performed except Difference-of-Normals filtering.
- `points_cluster` ([sensor_msgs/PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html))  
Pointcloud colored according to cluster.
- `points_ground` ([sensor_msgs/PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html))  
Pointcloud of only ground points.

## ROS Parameters

See the yaml file in the `config` folder for all ROS parameters and their descriptions
