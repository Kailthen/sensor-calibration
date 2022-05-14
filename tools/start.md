export BAG=../UrbanNav/UrbanNav-HK-Medium-Urban-1.bag
rosrun pcl_ros bag_to_pcd $BAG /velodyne_points pcds
python3 ./UrbanNavDataset/bag2img.py $BAG ./imgs/ /zed2/camera/left/image_raw
# rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map xxx 100

# kitti to bag
https://github.com/alterzlw/kitti2bag
