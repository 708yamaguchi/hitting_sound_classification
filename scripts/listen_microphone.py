#!/usr/bin/env python

# This node connects to microphone and publish ROS msg

from cv_bridge import CvBridge
import numpy as np
import rospy
import pyaudio
from hitting_sound_classification.msg import Spectrum, Volume, Wave
import matplotlib.cm as cm
import os
import os.path as osp
from PIL import Image as Image_
import rospkg
from sensor_msgs.msg import Image
import sys


class ListenMicrophone:

    def __init__(self):
        # init rospy node
        rospy.init_node('listen_microphone', anonymous=True)
        self.p = pyaudio.PyAudio()
        # config for microphone
        self.microphone_name = rospy.get_param('/microphone/name', 'default')
        self.chunk = rospy.get_param('/microphone/chunk')  # chunk is like a buffer, each buffer will contain chunk samples
        self.length = rospy.get_param('/microphone/length')  # length relates hamming window range
        self.rate = rospy.get_param('/microphone/rate')
        self.channels = 1
        self.format = pyaudio.paFloat32
        # search for microphone
        self.device_index = True
        for index in range(0, self.p.get_device_count()):
            device_info = self.p.get_device_info_by_index(index)
            if u'default' in device_info['name']:
                self.device_index = device_info['index']
        if self.device_index is True:
            print('Cannot find audio device!')
            sys.exit()
        # config for fft
        self.f = np.fft.fftfreq(self.length, d=1.0/self.rate)
        self.data = np.zeros((self.length, self.channels))
        self.window = np.hamming(self.length)
        self.stream = self.p.open(format=self.format,
                                  channels=self.channels,
                                  rate=self.rate,
                                  input=True,
                                  output=False,
                                  input_device_index=self.device_index,
                                  frames_per_buffer=self.chunk)
        # config for spectrogram
        self.cutoff_rate = rospy.get_param('~cutoff_rate')
        self.hit_volume_thre = rospy.get_param('~hit_volume_threshold')
        self.visualize_data_length = min(
            int(self.length * self.cutoff_rate / self.rate), self.length/2)
        self.time_to_listen = rospy.get_param('~time_to_listen')
        self.queue_size = int(self.time_to_listen * (self.rate / self.length))
        self.wave_spec_queue = np.zeros((
            self.queue_size,
            self.visualize_data_length
            ))  # remove folding noise
        self.bridge = CvBridge()
        self.count_from_last_hitting = 0

        # config for saving spectrogram
        rospack = rospkg.RosPack()
        self.save_image = rospy.get_param('~save_image')
        self.target_class = rospy.get_param(
            '~target_class', 'unspecified_data')
        self.save_dir = osp.join(rospack.get_path(
            'hitting_sound_classification'), 'hitting_sound_data')
        self.image_save_dir = osp.join(
            self.save_dir, 'image', 'origin', self.target_class)
        if self.save_image and not os.path.exists(self.image_save_dir):
            os.makedirs(self.image_save_dir)

        # publisher
        self.wave_pub = rospy.Publisher(  # sound wave data, the length is self.length
            '/microphone/wave', Wave, queue_size=1)
        self.spectrum_pub = rospy.Publisher(  # sound spectrum, which is fft of wave data
            '/microphone/sound_spec', Spectrum, queue_size=1)
        self.vol_pub = rospy.Publisher(  # current volume
            '/microphone/volume', Volume, queue_size=1)
        self.spectrogram_pub = rospy.Publisher(
            '/microphone/spectrogram', Image)
        self.hit_spectrogram_pub = rospy.Publisher(
            '/microphone/hit_spectrogram', Image)

        # published msg
        self.wavemsg = Wave()
        self.specmsg = Spectrum()
        self.volmsg = Volume()
        self.imgmsg = Image()

    def process(self):
        stamp = rospy.Time.now()
        tmp = self.stream.read(self.chunk)  # sound input -> float32 array
        data = np.fromstring(tmp, np.float32)
        self.data = np.array(data)

        # calc wave
        wave = self.data*self.window
        self.wavemsg.wave = wave
        self.wavemsg.header.stamp = stamp

        # calc volume
        vol = np.sqrt(np.mean(self.data**2))  # effective value
        self.volmsg.volume = vol
        self.volmsg.header.stamp = stamp

        # calc spectrum
        spec = np.abs(np.fft.fft(wave))
        self.specmsg.spectrum = spec
        self.specmsg.header.stamp = stamp

        # calc spectrogram
        spec_data = np.array(spec[:self.visualize_data_length])  # remove folding noise
        self.wave_spec_queue = np.concatenate([self.wave_spec_queue, spec_data[None]])
        self.wave_spec_queue = self.wave_spec_queue[1:]  # add new element to the queue
        normalized_spec_data = self.wave_spec_queue / np.max(self.wave_spec_queue)
        jet_img = np.array(cm.jet(1 - normalized_spec_data)[:, :, :3] * 255, np.uint8)
        jet_img_transposed = jet_img.transpose(1, 0, 2)[::-1]
        imgmsg = self.bridge.cv2_to_imgmsg(jet_img_transposed, 'bgr8')

        # publish msg
        self.spectrum_pub.publish(self.specmsg)
        self.vol_pub.publish(self.volmsg)
        self.wave_pub.publish(self.wavemsg)
        self.spectrogram_pub.publish(imgmsg)
        if vol > self.hit_volume_thre:
            self.count_from_last_hitting = 0
        else:  # publish (save) hit_spectrogram a little after hitting
            if self.count_from_last_hitting == self.queue_size / 3:
                if self.save_image:
                    file_num = len(os.listdir(self.image_save_dir)) + 1  # start from 00001.npy
                    file_name = osp.join(self.image_save_dir, '{0:05d}.png'.format(file_num))
                    Image_.fromarray(jet_img_transposed[:, :, [2, 1, 0]]).save(file_name)  # bgr -> rgb
                    rospy.loginfo('save image: ' + file_name)

                self.hit_spectrogram_pub.publish(imgmsg)
        self.count_from_last_hitting += 1

    def destruct(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.process()
        except rospy.ROSInterruptException:
            self.destruct()


if __name__ == '__main__':
    lm = ListenMicrophone()
    lm.run()
