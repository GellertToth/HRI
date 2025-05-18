#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

def move_square():
    rospy.init_node('square_move', anonymous=True)
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    sound_pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    rate = rospy.Rate(10)

    move_cmd = Twist()

    for i in range(4):
        # Move forward
        rospy.loginfo(f"Moving forward - Side {i+1}")
        move_cmd.linear.x = 0.3
        move_cmd.angular.z = 0.0
        start = rospy.Time.now()
        while rospy.Time.now() - start < rospy.Duration(2.0):
            pub.publish(move_cmd)
            rate.sleep()

        # Announce turning
        say = SoundRequest()
        say.sound = SoundRequest.SAY
        say.command = SoundRequest.PLAY_ONCE
        say.arg = f"Turning to side {i + 2 if i < 3 else 1}"
        say.volume = 1.0
        sound_pub.publish(say)

        rospy.loginfo("Turning")
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.5
        start = rospy.Time.now()
        while rospy.Time.now() - start < rospy.Duration(3.2):
            pub.publish(move_cmd)
            rate.sleep()

    rospy.loginfo("Stopping")
    pub.publish(Twist())  # stop

if __name__ == '__main__':
    try:
        move_square()
    except rospy.ROSInterruptException:
        pass
