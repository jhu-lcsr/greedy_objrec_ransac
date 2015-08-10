# Greedy ObjRec RANSAC

Finds object poses in a point cloud and publishes them. Uses TUM-MVP object RANSAC code to do so.

## Using this code

Greedy is designed to be used as a part of a ROS nodelet pipeline. It takes in point clouds on one topic, with class labels assigned in the RGB channels of the image.
