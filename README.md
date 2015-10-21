# ros_csv_extraction
CSV Data Extraction Tool for ROS bag files. 
ROS (robot operating system) is a software system gaining popularity in robotics for control and automation. 

This repository provides a tool to extract data to CSV format for a number of ROS message types for analysis in other software.

See original post on this code at http://www.shanelynn.ie/csv-data-extraction-tool-for-ros-bag-files/

Feel free to augment and enjoy.

# ROS Message Types
Thus far, the data extraction tool is compatible with the following ROS message types:

sensor_msgs/Image
sensor_msgs/Imu
sensor_msgs/LaserScan
sensor_msgs/NavSatFix
gps_common/gpsVel
umrr_driver/radar_msg (this was a type used by the CRUISE vehicle (see http://www.shanelynn.ie/csv-data-extraction-tool-for-ros-bag-files/))

To install the data extraction tool, download the zip file, extract it somewhere on your ROS_PACKAGE_PATH, and run rosmake data_extraction before using.

# Usage
 * Extract all compatible topics in a bag file:

`rosrun data_extract extract_all.py -b <path_to_bag_file> -o <path_to_output_dir>`

 * Extract a single topic:

`rosrun data_extraction extract_topic.py -b <path_to_bag_file> -o <path_to_output_csv_file> -t <topic_name>`
