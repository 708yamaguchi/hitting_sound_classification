#!/usr/bin/env python
# -*- coding: utf-8 -*-

from classify_sound_spectrum import ClassifySoundSpectrum
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import numpy as np


class ClassifySoundSpectrumROS:
    def __init__(self):
        self.wave_strength_thre = 1.0
        self.length = rospy.get_param('/mini_microphone/length')
        self.rate = rospy.get_param('/mini_microphone/rate')
        self.cutoff_rate = rospy.get_param('~cutoff_rate')
        self.visualize_data_length = min(int(self.length * self.cutoff_rate / self.rate), self.length/2)
        self.css = ClassifySoundSpectrum()
        self.css.load_data(k=7)
        self.sub = rospy.Subscriber('/mini_microphone/sound_spec', Float32MultiArray, self.cb)
        self.pub = rospy.Publisher('/object_class_by_spectrum', String, queue_size=1)

    def cb(self, msg):
        spec_data = np.array(msg.data[:self.visualize_data_length])  # remove folding noise
        wave_strength = np.sqrt(np.mean(spec_data**2))
        if wave_strength > self.wave_strength_thre:
            class_name = self.css.predict(spec_data)
            msg = String()
            msg.data = class_name
            self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('classify_sound_spectrum_ros')
    ClassifySoundSpectrumROS()
    rospy.spin()
