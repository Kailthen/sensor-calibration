export BAG=/media/work/Elements/M2DGR/gate_03.bag
mkdir pcds 
rosrun pcl_ros bag_to_pcd $BAG /velodyne_points pcds
mkdir imgs
python3 ../../tools/bag2img.py $BAG ./imgs/ /cam_1/color/image_raw
# rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map xxx 100

# kitti to bag
https://github.com/alterzlw/kitti2bag


python3 tools/bag_parser.py  --topic /livox/lidar /camera/image_color/compressed \
    --output_dir ./datas/r3live/pcds ./datas/r3live/imgs \
    --bag_file /media/work/Elements/r3live/hku_park_01.bag \
    --start_time 1627719660 --end_time  1627719662

python3 tools/bag_parser.py  --topic /velodyne_points /camera/image_color \
    --output_dir ./datas/UrbanNav-HK-Data20200314/pcds ./datas/UrbanNav-HK-Data20200314/imgs \
    --bag_file /media/work/Elements/UrbanNav/2020-03-14-16-45-35.bag \
    --start_time 1584175536 --end_time  1584175537

