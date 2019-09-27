# trafficlight_recognizer

This package contains tools and nodes that are related to traffic light recognition.

## feat_proj

## feat_proj_lanelet2

### Feature
This node projects traffic light in Lanelet2 map into camera image.
Projected information is assumed be used in traffic light recognition nodes (`region_tlr_*` nodes). 

### Map Requirements
Your Lanelet2 map should contain traffic light regulatory elements to use this feature.

Ideally, traffic light regulatory element should have optional `light_bulbs` members as explained in "Format Extension for Autoware" in order to project accurate position of each lamp. If `light_bulbs` member is not present(e.g. if you are using default lanelet2 format), this node will assume that lamps are at each corner of traffic light. This allows the following traffic recognition nodes to calculate bounding box of traffic light in the image. In case you do not have `light_bulbs` defined in your map, you can only use either `region_tlr_mxnet` or `region_tlr_ssd` for color recognition, not `region_tlr`

### How to Run
Using rosrun:
```
rosrun trafficlight_recognizer feat_proj_lanelet2
```
Using roslaunch:
```
roslaunch trafficlight_recognizer feat_proj_lanelet2.launch
```

### parameters

|Parameter| Type| Description|Default|
----------|-----|--------|---|
|`~/use_path_info`|*bool*|When this is set `true`, only traffic lights associated to current driving lane are projected. Otherwise, all visible traffic lights are projected.|`false`|
|`~/camera_frame`|*String*|Name of optical frame of the camera.|`"camera"`|

### subscribed topics

|topic| type | Description|
----------|-----|--------|
|`/lanelet_map_bin`|*lanelet_msgs/MapBin*|binary data of lanelet2 map|
|`/camera_info`|*sensor_msgs/CameraInfo*|Camera parameters used for projection|
|`/config/adjust_xy`|*autoware_msgs/AdjustXY*|This topic sets offsets of projection in image|
|`/final_waypoints`|*autoware_msgs/Lane*|path of vehicle. only subscribed when `use_path_info` parameter is set `true`|

### published topics

|topic| type | Description|
----------|-----|--------|
|`/roi_signal`|*autoware_msgs/Signals*|Contains positions of traffic lights in both map frame and image frame|

### required tf
map->camera

## label_maker

## region_tlr

## region_tlr_mxnet

## region_tlr_ssd

## tl_switch

## tlr_tuner
