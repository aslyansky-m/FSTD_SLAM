
#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
	"""Extract a folder of images from a rosbag.
	"""
	parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
	parser.add_argument("bag_file", help="Input ROS bag.")
	parser.add_argument("output_dir", help="Output directory.")
	parser.add_argument("image_topic", help="Image topic.")

	args = parser.parse_args()

	print("Extract images from {} on topic {} into {}".format(args.bag_file,
		                          args.image_topic, args.output_dir))

	bag = rosbag.Bag(args.bag_file, "r")
	bridge = CvBridge()
	count = 0

	(path, filename)  = os.path.split(args.bag_file)

	voObj = None

	compressed = False

	for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
		
		if compressed :
			try:
				cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
			except:
				compressed = True
		else:
			cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

		if(voObj is None):
			fourcc = cv2.VideoWriter_fourcc('F','M','P','4') #cv2.VideoWriter_fourcc('X264')
			height, width, channels = cv_img.shape
			voObj = cv2.VideoWriter(os.path.join(args.output_dir,filename+'.mp4'),fourcc, 60.0, (width,height))
		cv2.imshow('Example', cv_img)
		cv2.waitKey(1)
		
		voObj.write(cv_img)

		count += 1
	print('saved to ' + os.path.join(args.output_dir,filename+'.mp4'))
	bag.close()
	voObj.release()
	return

if __name__ == '__main__':
    main()
