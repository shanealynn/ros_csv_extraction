#!/usr/bin/env python

import roslib 
roslib.load_manifest('data_extraction')
import rospy
import rosbag
import os 		#used to get directory for image topics
import sys 		#used for errors and exiting
import getopt 	#used to parse arguments
import subprocess, yaml #required to check bag file contents

def usage():
 	print " -----------------------------------  USAGE:  -------------------------------------------------"
 	print " rosrun data_extraction extract_all.py -b rosbag_file_name -o output_dir_name"
 	print ""
 	print " This code will extract all supported message types from the specified rosbag file. "
 	print ""
 	print "  rosbag_file_name - path to ros .bag file containing ROS records"
 	print "  output_dir_name  - full path directory to save record to. Images are saved to this directory "
 	print "                     also. Directory must exist prior to running code.  "
 	print "  "
 	print "  Currently supported message types are:"
 	print "                   - sensor_msgs/Image "
 	print "                   - sensor_msgs/Imu "
 	print "                   - sensor_msgs/LaserScan "
 	print "                   - sensor_msgs/NavSatFix "
 	print "                   - gps_common/gpsVel  "
 	print "                   - umrr_driver/radar_msg  "
 	print ""
 	print "  Rosbag Extraction Script v1 - Shane Lynn - 7th May 2012"
 	print ""
 	print " ---------------------------------------------------------------------------------------------"

allowedTopics = ['sensor_msgs/Image', 'sensor_msgs/Imu', 'sensor_msgs/LaserScan', 'sensor_msgs/NavSatFix', \
			'gps_common/GPSFix', 'umrr_driver/radar_msg']	

def main():
	rospy.loginfo("Processing input arguments:")
	try:
		opts, extraparams = getopt.getopt(sys.argv[1:], "o:b:t:") #start at the second argument.
	except getopt.GetoptError, err:
		#print error info and exit
		print str(err)
		usage()
		sys.exit(2)
	
	#default values
	outDir = "output"
	rosbagFile = "bagfile.bag"	
	
	for o,a in opts:
		if o == "-o":
			outDir = a
		elif o == "-b":
			rosbagFile = a
		else:
			assert False, "unhandled option"
			usage()
			
	rospy.loginfo("Opening bag file: " + rosbagFile)
	try:
		bag = rosbag.Bag(rosbagFile)
	except:
		rospy.logerr("Error opening specified bag file : %s"%rosbagFile)
		usage()
		sys.exit(2)
	
	rospy.loginfo ("Bag file opened.")
	
	rospy.loginfo("Scanning topics...")
	
	infoDict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', rosbagFile], stdout=subprocess.PIPE).communicate()[0])
		
	#we now have all of the topics contained in infoDict. We need to run through these and run the data extract script.
	for ii in range(len(infoDict['topics'])):		
		topicType = infoDict['topics'][ii]['type']	
		topicName = infoDict['topics'][ii]['topic']		
		#is this an processable topic?
		rospy.loginfo("Found topic " + topicName + " of type " + topicType)
		process = False
													
		for object in allowedTopics:
			if object == topicType:
				process = True
				break									
		
		if process == True:
			rospy.loginfo("Processing topic...")
			rospy.loginfo("----------------------- Starting TOPIC_EXTRACT.PY for %s --------------------------"%topicName)
			fileName =  topicName.replace("/", "-")			
			commandLine = "rosrun data_extraction extract_topic.py -b " + rosbagFile + " -o " + outDir + "/" + fileName + ".csv" + " -t " + topicName		
			os.system(commandLine)
			rospy.loginfo("----------------------- Finished TOPIC_EXTRACT.PY for %s --------------------------"%topicName)
		else:
			rospy.loginfo("Topic Type not supported.")
														 
	#Summarise for user								    		
	bag.close()
	rospy.loginfo("Completed for all compatible topics.")
	
main()
