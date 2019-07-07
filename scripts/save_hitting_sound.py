#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from cv_bridge import CvBridge
import matplotlib.cm as cm
from sensor_msgs.msg import Image
from PIL import Image as Image_

import os
import os.path as osp


class SaveHittingSound:
    def __init__(self):
        rospy.init_node('save_hitting_sound', anonymous=True)
        self.length = rospy.get_param('/mini_microphone/length')
        self.rate = rospy.get_param('/mini_microphone/rate')
        self.cutoff_rate = rospy.get_param('~cutoff_rate')
        self.wave_strength_thre = 1.0
        self.visualize_data_length = min(int(self.length * self.cutoff_rate / self.rate), self.length/2)
        self.time_to_listen = rospy.get_param('~time_to_listen')
        self.queue_size = int(self.time_to_listen * (self.rate / self.length))
        self.wave_spec_queue = np.zeros((
            self.queue_size,
            self.visualize_data_length
            ))  # remove folding noise
        # publish
        self.hitting_sound_pub = rospy.Publisher(
            '/mini_microphone/hitting_sound', Float32MultiArray)
        self.sound_image_pub = rospy.Publisher(
            '/mini_microphone/sound_image', Image)
        self.hitting_sound = Float32MultiArray()
        self.hitting_sound.layout = []
        dim = MultiArrayDimension()
        dim.label = 'time'
        dim.size = self.queue_size
        dim.stride = self.queue_size * self.length
        self.hitting_sound.layout.append(dim)
        dim.label = 'wave_spec'
        dim.size = self.length
        dim.stride = self.length
        self.hitting_sound.layout.append(dim)
        self.bridge = CvBridge()
        self.count_from_last_hitting = 0
        # subscribe
        rospy.Subscriber('/mini_microphone/sound_spec', Float32MultiArray, self.sound_spec_cb)
        # save data
        self.save_image = rospy.get_param('~save_image')
        self.save_spectrum = rospy.get_param('~save_spectrum')
        self.target_class = rospy.get_param('~target_class', 'unspecified_data')
        self.save_dir = osp.join(os.environ['HOME'], 'hitting_sound_data')
        self.spectrum_save_dir = osp.join(self.save_dir, 'spectrum', self.target_class)
        if self.save_spectrum and not os.path.exists(self.spectrum_save_dir):
            os.makedirs(self.spectrum_save_dir)
        self.image_save_dir = osp.join(self.save_dir, 'image', 'origin', self.target_class)
        if self.save_image and not os.path.exists(self.image_save_dir):
            os.makedirs(self.image_save_dir)

    def sound_spec_cb(self, msg):
        spec_data = np.array(msg.data[:self.visualize_data_length])  # remove folding noise
        self.wave_spec_queue = np.concatenate([self.wave_spec_queue, spec_data[None]])
        self.wave_spec_queue = self.wave_spec_queue[1:]
        # publish hitting sound
        self.hitting_sound.data = self.wave_spec_queue.flatten()
        self.hitting_sound_pub.publish(self.hitting_sound)
        # publish hitting sound visualization
        normalized_spec_data = self.wave_spec_queue / np.max(self.wave_spec_queue)
        jet_img = np.array(cm.jet(1 - normalized_spec_data)[:, :, :3] * 255, np.uint8)
        jet_img_transposed = jet_img.transpose(1, 0, 2)[::-1]
        imgmsg = self.bridge.cv2_to_imgmsg(jet_img_transposed, 'bgr8')
        self.sound_image_pub.publish(imgmsg)
        # save spectrum if the hitting is strong
        wave_strength = np.sqrt(np.mean(spec_data**2))
        if wave_strength > self.wave_strength_thre:
            self.count_from_last_hitting = 0
            if self.save_spectrum:
                file_num = len(os.listdir(self.spectrum_save_dir)) + 1  # start from 00001.npy
                file_name = osp.join(self.spectrum_save_dir, '{0:05d}.npy'.format(file_num))
                np.save(file_name, spec_data)
                rospy.loginfo('save spectrum: ' + file_name)
        else:  # save image a little after hitting
            if self.save_image and (self.count_from_last_hitting == self.queue_size / 3):
                file_num = len(os.listdir(self.image_save_dir)) + 1  # start from 00001.npy
                file_name = osp.join(self.image_save_dir, '{0:05d}.png'.format(file_num))
                # np.save(file_name, jet_img_transposed)
                Image_.fromarray(jet_img_transposed).save(file_name)
                rospy.loginfo('save image: ' + file_name)
        self.count_from_last_hitting += 1


if __name__ == '__main__':
    shs = SaveHittingSound()
    rospy.spin()
