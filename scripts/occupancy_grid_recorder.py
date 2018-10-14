#! /usr/bin/python

import sys
import os
import subprocess
import datetime

import rospy


def occupancy_grid_recorder():

	rospy.init_node('occupancy_grid_recorder', anonymous=True)


	interval_default = 10
	bag_file_name_default = "occupancy_grid_" + datetime.datetime.now().isoformat() + ".bag"
	bag_file_dir_default = os.path.join( os.path.dirname(__file__), "../bags/")

	interval = interval_default
	bag_file_name = bag_file_name_default
	bag_file_dir = bag_file_dir_default

	if len(sys.argv) >= 2:
		try:
			interval = int(sys.argv[1])
		except TypeError as e:
			rospy.logerr("occupancy_grid_recorder: Could not parse interval. Using default value")
			interval = interval_default

	if len(sys.argv) >= 3:
		bag_file_name = sys.argv[2]

	if len(sys.argv) >= 4:
		bag_file_dir = sys.argv[3]


	rate = rospy.Rate(1.0/interval) # 10hz
	
	bag_file_path = os.path.join(bag_file_dir, bag_file_name)
	rospy.loginfo("occupancy_grid_recorder: Using bagfile " + bag_file_path)


	if subprocess.call(["touch", bag_file_path]) != 0:
		rospy.logwarn("occupancy_grid_recorder: Unable to open file" + bag_file_path)

		bag_file_path = os.path.join(bag_file_dir_default, bag_file_name_default)
		if subprocess.call(["touch", bag_file_path]) != 0:
			rospy.logwarn("occupancy_grid_recorder: Unable to open default file" + bag_file_path)

			rospy.signal_shutdown("occupancy_grid_recorder: Unable to open default file" + bag_file_path)


	rosbag_process = None

	while not rospy.is_shutdown():

		if rosbag_process:
			rosbag_process.send_signal(subprocess.signal.SIGINT)

		try:
			rosbag_process = subprocess.Popen(["rosbag", "record", "map", "map_metadata", "-O", bag_file_path, "-l", "1"])
		except OSError as e:
			rospy.logerr("occupancy_grid_recorder: Map not recorded, rosbag failed")
		else:
			rospy.loginfo("occupancy_grid_recorder: Map recorded")

		rate.sleep()


if __name__ == '__main__':
	try:
		occupancy_grid_recorder()
	except rospy.ROSInterruptException:
		pass
