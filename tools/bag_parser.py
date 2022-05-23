#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import argparse

import numpy as np
import cv2
from pyrsistent import field

import rosbag
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
from rospy import Time

import sensor_msgs.point_cloud2 as pc2
from pc_utils import pointcloud2_to_array

import sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
print(BASE_DIR)
sys.path.append(BASE_DIR)

from pypcd import pypcd

def main(args):
    """
        Extract .
    """
    start_time = None
    end_time = None
    if args.start_time:
        start_time = Time.from_seconds(args.start_time)
    if args.end_time:
        end_time = Time.from_seconds(args.end_time)

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()

    for topic, msg, t in bag.read_messages(topics=args.topic, start_time=start_time, end_time=end_time):

        if "CompressedImage" in str(type(msg)):
            cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
            tfile = os.path.join(args.topic_dir[topic], "{:.9f}.jpg".format(msg.header.stamp.to_sec()))
            cv2.imwrite(tfile, cv_img)
            print("Save CompressedImage to {}".format(tfile))
        elif "Image" in str(type(msg)): #isinstance(msg, Image):
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            tfile = os.path.join(args.topic_dir[topic], "{:.9f}.jpg".format(msg.header.stamp.to_sec()))
            cv2.imwrite(tfile, cv_img)
            print("Save Image to {}".format(tfile))
        elif "PointCloud2" in str(type(msg)):
            pcd_arr = pointcloud2_to_array(msg)
            pcd_file = os.path.join(args.topic_dir[topic], "{:.9f}.arr".format(msg.header.stamp.to_sec()))
            # pypcd.save_point_cloud_bin_compressed(pcd, pcd_file)
            np.save(pcd_file, pcd_arr, allow_pickle=True)
            print("Save pcd to {}".format(pcd_file))

        elif "CustomMsg" in str(type(msg)):
            from livox_ros_driver.msg import CustomMsg
            # convert to numpy array
            timebase = msg.timebase # ns
            point_num = msg.point_num

            fields_dict = {"offset_time":[], 
                "x":[],
                "y":[],
                "z":[],
                "reflectivity":[],
                "tag":[],
                "line":[],
                }
            for p in msg.points:
                # uint32 offset_time      # offset time relative to the base time
                # float32 x               # X axis, unit:m
                # float32 y               # Y axis, unit:m
                # float32 z               # Z axis, unit:m
                # uint8 reflectivity      # reflectivity, 0~255 
                # uint8 tag               # livox tag
                # uint8 line              # laser number in lidar
                fields_dict["offset_time"].append(p.offset_time)
                fields_dict["x"].append(p.x)
                fields_dict["y"].append(p.y)
                fields_dict["z"].append(p.z)
                fields_dict["reflectivity"].append(p.reflectivity)
                fields_dict["tag"].append(p.tag)
                fields_dict["line"].append(p.line)

            fields = [
                fields_dict["x"],
                fields_dict["y"],
                fields_dict["z"],
                fields_dict["reflectivity"],
                fields_dict["line"],
                fields_dict["offset_time"]
            ]
            # dtypes = [
            #     ('x', 'float32'),
            #     ('y', 'float32'),
            #     ('z', 'float32'),
            #     ('i', 'uint8'),
            #     ('ring', 'uint8'),
            #     ('time', 'uint32')
            # ]
            pcd_arr = np.array(fields, dtype=np.float32)
            # pcd = pypcd.PointCloud.from_array(pcd_arr)

            pcd_file = os.path.join(args.topic_dir[topic], "{:.9f}.arr".format(msg.header.stamp.to_sec()))
            # pypcd.save_point_cloud_bin_compressed(pcd, pcd_file)
            np.save(pcd_file, pcd_arr, allow_pickle=True)
            print("Save pcd to {}".format(pcd_file))

    bag.close()

    return


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("--bag_file", help="Input ROS bag.", required=True)
    parser.add_argument("--topic", nargs="+", help="One or more topics.", required=True)
    parser.add_argument("--output_dir", nargs="+", help="Output directories.", required=True)

    parser.add_argument("--start_time", type=float, required=False, default=None)
    parser.add_argument("--end_time", type=float, required=False, default=None)

    args = parser.parse_args()

    if len(args.topic) != len(args.output_dir):
        print("Error params.")
        exit(0)
    
    topic_dir = {}
    for topic, dr in zip(args.topic, args.output_dir):
        os.makedirs(dr, exist_ok=True)
        topic_dir[topic] = dr
    args.topic_dir = topic_dir

    main(args)