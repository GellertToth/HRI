#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class VoiceControlledMover:
    def __init__(self):
        rospy.init_node('voice_controlled_mover', anonymous=True)

        # Publishers
        self.cmd_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        self.sound_pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)

        # Subscriber
        rospy.Subscriber('/speech_text', String, self.command_callback)

        self.rate = rospy.Rate(10)
        self.move_cmd = Twist()
        rospy.loginfo("VoiceControlledMover initialized. Say 'left', 'right', or 'forward'.")

    def say(self, text):
        say = SoundRequest()
        say.sound = SoundRequest.SAY
        say.command = SoundRequest.PLAY_ONCE
        say.arg = text
        say.volume = 1.0
        self.sound_pub.publish(say)

    def move(self, linear_x=0.0, angular_z=0.0, duration=2.0):
        self.move_cmd.linear.x = linear_x
        self.move_cmd.angular.z = angular_z
        start = rospy.Time.now()
        while rospy.Time.now() - start < rospy.Duration(duration) and not rospy.is_shutdown():
            self.cmd_pub.publish(self.move_cmd)
            self.rate.sleep()
        self.cmd_pub.publish(Twist())  # Stop after motion

    def command_callback(self, msg):
        command = msg.data.lower().strip()
        rospy.loginfo(f"Received command: {command}")

        if command == "forward":
            self.say("Moving forward")
            self.move(linear_x=0.3, duration=2.0)

        elif command == "left":
            self.say("Turning left")
            self.move(angular_z=0.5, duration=3.0)

        elif command == "right":
            self.say("Turning right")
            self.move(angular_z=-0.5, duration=3.0)

        else:
            rospy.loginfo("Unknown command. Please say 'left', 'right', or 'forward'.")

if __name__ == '__main__':
    try:
        mover = VoiceControlledMover()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
