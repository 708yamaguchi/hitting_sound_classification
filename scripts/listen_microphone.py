#!/usr/bin/env python

import numpy as np
import rospy
import pyaudio
from std_msgs.msg import Float32, Float32MultiArray
import sys


class ListenMiniMicrophone:

    def __init__(self):
        # init rospy node
        rospy.init_node('listen_microphone', anonymous=True)
        self.p = pyaudio.PyAudio()
        # config for mini microphone
        self.microphone_name = rospy.get_param('/microphone/name', 'default')
        self.chunk = rospy.get_param('/microphone/chunk')  # chunk is like a buffer, each buffer will contain chunk samples
        self.length = rospy.get_param('/microphone/length')  # length relates hamming window range
        self.rate = rospy.get_param('/microphone/rate')
        self.channels = 1
        self.format = pyaudio.paFloat32
        # set parameters for PnP Sound Device
        self.device_index = True
        for index in range(0, self.p.get_device_count()):
            device_info = self.p.get_device_info_by_index(index)
            # if u'USB PnP Sound Device' in device_info['name']:
            if u'default' in device_info['name']:
                self.device_index = device_info['index']
        if self.device_index is True:
            print('Cannot find audio device!')
            sys.exit()
        # variables for fft
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
        # sound wave data, the length is self.length
        self.wavepub = rospy.Publisher(
            '/microphone/wave', Float32MultiArray, queue_size=1)
        # sound spectrum, which is fft of wave data
        self.specpub = rospy.Publisher(
            '/microphone/sound_spec', Float32MultiArray, queue_size=1)
        # current volume
        self.volpub = rospy.Publisher(
            '/microphone/volume', Float32, queue_size=1)
        self.wavemsg = Float32MultiArray()
        self.specmsg = Float32MultiArray()
        self.volmsg = Float32()

    def process(self):
        # sound input -> float32 array
        tmp = self.stream.read(self.chunk)
        data = np.fromstring(tmp, np.float32)
        self.data = np.array(data)
        wave = self.data*self.window
        vol = np.sqrt(np.mean(self.data**2))  # effective value
        spec = np.fft.fft(wave)

        # msg
        # self.wavemsg.data = wave[-self.chunk:, i].tolist()
        self.wavemsg.data = wave
        # self.specmsg.data = np.abs(spec[:, i]).tolist()
        self.specmsg.data = np.abs(spec)
        # self.specmsg.phase = np.angle(spec[:, i]).tolist()  # lost angle data
        self.volmsg.data = vol
        # publish msg
        self.wavepub.publish(self.wavemsg)
        self.specpub.publish(self.specmsg)
        self.volpub.publish(self.volmsg)

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
    lmm = ListenMiniMicrophone()
    lmm.run()
