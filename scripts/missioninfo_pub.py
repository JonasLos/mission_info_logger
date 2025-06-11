#!/usr/bin/env python

import rospy
from mission_info_logger.msg import MissionInfo  # Import your custom message

def main():
    rospy.init_node('parameter_logger_node')

    # Get parameters
    location = rospy.get_param('~location', 'Unknown location')
    vehicle = rospy.get_param('~vehicle', 'Unknown vehicle')
    comments = rospy.get_param('~comments', 'No comments')
    maneuver = rospy.get_param('~maneuver', 'No maneuver specified')

    # Log
    rospy.loginfo("=== Parameter Logger Node ===")
    rospy.loginfo("Location: %s", location)
    rospy.loginfo("Vehicle: %s", vehicle)
    rospy.loginfo("Comments: %s", comments)
    rospy.loginfo("Maneuver: %s", maneuver)

    # Publisher
    pub = rospy.Publisher('mission_info', MissionInfo, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    msg = MissionInfo()
    msg.location = location
    msg.vehicle = vehicle
    msg.comments = comments
    msg.maneuver = maneuver

    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.loginfo("Published mission info")
        rate.sleep()

if __name__ == '__main__':
    main()

