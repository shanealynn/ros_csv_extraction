#!/usr/bin/env python

import roslib 
roslib.load_manifest('data_extraction')
import rospy
import rosbag
import cv
from cv_bridge import CvBridge, CvBridgeError
#from sensor_msgs.msg import Image
import csv		#writing CV files.
import os 		#used to get directory for image topics
import sys 		#used for errors and exiting
import getopt 	#used to parse arguments

def usage():
 	print " -----------------------------------  USAGE:  -------------------------------------------------"
 	print " rosrun data_extraction extract_topic.py -b bag_file_name -o output_file_name -t topic_name"
 	print ""
 	print "  bag_file_name - path to ros .bag file containing ROS records"
 	print "  output_file_name - full path to csv file to save records to. If an image topic is specified"
 	print "                     then the images will be saved to the same directory as the output file " 
 	print "                     and named Image_0001.jpg, Image_0002.jpg, etc."
 	print "  topic_name       - topic in bag file that contains ROS messages."
 	print "  "
 	print "  Currently supported message types are:"
 	print "                   - sensor_msgs/Image "
 	print "                   - sensor_msgs/Imu "
 	print "                   - sensor_msgs/LaserScan "
 	print "                   - sensor_msgs/NavSatFix "
 	print "                   - gps_common/gpsVel  "
 	print "                   - umrr_driver/radar_msg  "
 	print ""
 	print "  Rosbag Extraction Script v1 - Shane Lynn - 3rd May 2012"
 	print ""
 	print " ---------------------------------------------------------------------------------------------"


def getHeader(msg):
	#this function makes up the top of the csv file
	msgType = str(type(msg))	
	msgType = msgType[msgType.index('.')+1:]	
	msgType = msgType[:msgType.index('\'')]
	
	rospy.loginfo("Messages are of type %s"%msgType)
	rospy.loginfo("Processing records:")
	isImage = False
	
	#Add handlers as necessary for each message type:
	
	if (msgType == '_sensor_msgs__LaserScan'): 		
		headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
						 "angle_min", "angle_max", "angle_increment","time_increment", "scan_time", "range_min", "range_max" ]
		#set up the header row:
		for i in range(len(msg.ranges)):
			headerRow.append("Range%s"%(i+1))
		if len(msg.intensities) >= 1:
			for i in range(len(msg.intensities)):
				headerRow.append("Intensity%s"%(i+1))
				
	elif (msgType == '_sensor_msgs__Imu'):
		headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
		                 "Orientation_x", "Orientation_y", "Orientation_z", "Orientation_w", \
		                 "AngularVelocity_x", "AngularVelocity_y", "AngularVelocity_z", \
		                 "LinearAcceleration_x", "LinearAcceleration_y", "LinearAcceleration_z"]
		
	elif (msgType == '_sensor_msgs__NavSatFix'):
		headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
		                 "Latitude", "Longitude", "Altitude", "PositionCovariance"]
		
	elif (msgType == '_gps_common__GPSFix'):
		headerRow =["Time", "Header_sequence", "Header_secs", "Header_nsecs", "GPS_status", \
		                 "Latitude", "Longitude", "Altitude", "PositionCovariance", "PositionCovariance_type", \
		                 "Track", "Speed", "Vert_Speed", "GPS_time", "Pitch", "Roll", "Dip" \
		                 ]
	elif (msgType == '_sensor_msgs__Image'):
		headerRow = ["Time", "Header_ sequence", "Header_secs", "Header_nsecs", "Filename"]
		isImage = True
		
	elif (msgType == '_umrr_driver__radar_msg'):
		headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
					"SensorID", "NumTargets", "Mode", "Submode", "Status", \
					"Target_Range", "Target_Angle", "Target_RadialSpeed", "Target_Signal2Threshold", "Target_Type"]
		
	else:
		rospy.logerr("Unsupport Message type %s"%msgType)
		usage()
		sys.exit(2)
	
	return headerRow, isImage


def getColumns(t, msg, imageFile = ""):
	#this function gets the data that is necessary for the csv file - one row at a time from the current msg.
	msgType = str(type(msg))	
	msgType = msgType[msgType.index('.')+1:]	
	msgType = msgType[:msgType.index('\'')]	
	
	if (msgType == '_sensor_msgs__LaserScan'): 
		columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, \
		                     msg.angle_min, msg.angle_max, msg.angle_increment, msg.time_increment, msg.scan_time, msg.range_min, msg.range_max ]
		for i in range(len(msg.ranges)): 
			columns.append(msg.ranges[i])
		
		if len(msg.intensities) >= 1:
			for i in range(len(msg.intensities)): 
				columns.append(msg.intensities[i])
				
	elif (msgType == '_sensor_msgs__Imu'):
		columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, \
		                     msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w, \
		                     msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, \
		                     msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
		
	elif (msgType == '_sensor_msgs__NavSatFix'):
		columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, \
		                     msg.latitude, msg.longitude, msg.altitude, msg.position_covariance]
		
	elif (msgType == '_gps_common__GPSFix'):
		columns =[t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, msg.status.status, \
		                     msg.latitude, msg.longitude, msg.altitude, msg.position_covariance, msg.position_covariance_type, \
		                     msg.track, msg.speed, msg.climb, msg.time, msg.pitch, msg.roll, msg.dip ]
	elif (msgType == '_sensor_msgs__Image'):
		columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, imageFile]
	
	elif (msgType == '_umrr_driver__radar_msg'):
		columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, \
					msg.sensorID, msg.numTargets, msg.mode, msg.submode, msg.status, \
					msg.targetRange, msg.targetAngle, msg.targetRadialSpeed, msg.targetSig2Threhold, msg.targetType ]
	
	else:
		rospy.logerror("Unexpected error - AGH!")
		usage()
		sys.exit(2)
				
	return columns
	

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
	outFile = "output.csv"
	rosbagFile = "bagfile.bag"
	topicName = "/crossbow_imu/data"
	
	for o,a in opts:
		if o == "-o":
			outFile = a
		elif o == "-b":
			rosbagFile = a
		elif o == "-t":
			topicName = a
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

	rospy.loginfo("Opening " + outFile + " for writing..")
	try:
		fileH = open(outFile, 'wt')
		fileWriter = csv.writer(fileH)
	except:
		rospy.logerr("Error opening specified output file : %s"%outFile)
		usage()
		sys.exit(2)
		
	rospy.loginfo ("Output file opened.")
	#get the directory if we need it
	outDir = os.path.dirname(outFile)

	rospy.loginfo("Getting topic " + topicName + " from bag file.")
	count = 1
	isImage = False	
	imageFile = ""
	
	#cvBridge set up in case we have images to deal with
	bridge = CvBridge()
		
	#fileWriter.writerow(["Time", "Header sequence", "Header secs", "Header nsecs", \
	#"angle_min", "angle_max", "angle_increment","time_increment", "scan_time", "range_min", "range_max",\
	#						 "ranges", "intensities" ])
	for topic, msg, t in bag.read_messages(topics=topicName):
		#on the first message, we need to set the header...        
		if count == 1:								
			#use this type to process the headers:
			headerRow, isImage = getHeader(msg)
			if headerRow != None:						
				fileWriter.writerow(headerRow)					
									
		#if we are dealing with image data - we also need to write the image file.
		if isImage:			
			try:
				cvImage = bridge.imgmsg_to_cv(msg, "bgr8")
			except CvBridgeError, err:
				print err
			
			imageFile = 'Image_%.4d'%count			
			saveFileName = str(outDir)+"/"+imageFile+".jpg"
			cv.SaveImage(saveFileName, cvImage)
			
		#get the columns for the csv file.
		columns = getColumns(t, msg, imageFile)
		
		#write the columns or image to the file/folder.
		fileWriter.writerow(columns)
		
		#keep track of the number of records processed
		count = count + 1
		if (count % 100 == 0):
			rospy.loginfo("Processed %s records..."%count)
				
														  
	#Summarise for user								    
	rospy.loginfo("GRAND TOTAL of %s records."%count)
	if count < 5:
		rospy.logwarn("WARNING: Very few records processed - is your topic name correctly specified?")
	rospy.loginfo("Closing files and cleaning up.")
	fileH.close()
	bag.close()
	rospy.loginfo("Done!")
	
main()
