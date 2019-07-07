#!/usr/bin/env python

import argparse
import numpy as np
import os.path as osp
import os

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import chainer
from chainer import cuda
from chainer import dataset
from chainer import training
from chainer.training import extensions
from chainer_modules import alex
from chainer_modules import googlenet
from chainer_modules import googlenetbn
from chainer_modules import nin
from chainer_modules import resnet50
from chainer_modules import resnext50

from train import PreprocessedDataset

from PIL import Image as Image_

class ClassifySoundImageROS:
    def __init__(self):
        archs = {
            'alex': alex.Alex,
            'googlenet': googlenet.GoogLeNet,
            'googlenetbn': googlenetbn.GoogLeNetBN,
            'nin': nin.NIN,
            'resnet50': resnet50.ResNet50,
            'resnext50': resnext50.ResNeXt50,
        }

        self.gpu = 0
        device = chainer.cuda.get_device(self.gpu)  # for python2, gpu number is 0

        print('Device: {}'.format(device))
        print('Dtype: {}'.format(chainer.config.dtype))
        print('')

        # Initialize the model to train
        n_class_file = osp.join(os.environ['HOME'], 'hitting_sound_data', 'image', 'dataset', 'n_class.txt')
        n_class = 0
        self.classes = []
        with open(n_class_file, mode='r') as f:
            for row in f:
                self.classes.append(row.strip())
                n_class += 1
        self.model = archs['nin'](n_class=n_class)
        initmodel = rospy.get_param('~model')
        print('Load model from {}'.format(initmodel))
        chainer.serializers.load_npz(initmodel, self.model)
        self.model.to_device(device)
        device.use()

        # Load the mean file
        mean_file_path = osp.join('/'.join(initmodel.split('/')[:-2]), 'chainer_modules', 'mean.npy')
        self.mean = np.load(mean_file_path)
        # # Load the dataset files
        # train = PreprocessedDataset(args.train, args.root, mean, model.insize)
        # val = PreprocessedDataset(args.val, args.root, mean, self.model.insize,
        #                           False)
        # # These iterators load the images with subprocesses running in parallel
        # # to the training/validation.
        # train_iter = chainer.iterators.MultiprocessIterator(
        #     train, args.batchsize, n_processes=args.loaderjob)
        # val_iter = chainer.iterators.MultiprocessIterator(
        #     val, args.val_batchsize, repeat=False, n_processes=args.loaderjob)
        # converter = dataset.concat_examples

        # Set up an optimizer
        optimizer = chainer.optimizers.MomentumSGD(lr=0.01, momentum=0.9)
        optimizer.setup(self.model)

        self.sub = rospy.Subscriber(
            '/mini_microphone/sound_image', Image, self.cb)
        self.pub = rospy.Publisher('/object_class_by_image', String, queue_size=1)
        self.bridge = CvBridge()

    def cb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        with chainer.using_config('train', False), \
             chainer.no_backprop_mode():
            x_data = np.array(Image_.fromarray(cv_image).resize((256, 256))).astype(np.float32)
            x_data = x_data.transpose((2, 0, 1))
            ############### kokode itti with train.py
            # import cv2
            # cv2.imshow('hoge', x_data.transpose((1, 2, 0)).astype(np.uint8))
            # cv2.waitKey(0)
            ###############
            # substruct mean value (but I do not have confidence in the code ...)
            # ch_mean = np.average(self.mean, axis=(1, 2)).astype(np.float32)
            mean = self.mean.astype(np.float32)
            x_data -= mean
            x_data *= (1.0 / 255.0)  # Scale to [0, 1]
            # fowarding once
            x_data = cuda.to_gpu(x_data[None], device=self.gpu)
            x_data = chainer.Variable(x_data)
            ret = self.model.forward_for_test(x_data)
            ret = cuda.to_cpu(ret.data)[0]
        msg = String()
        msg.data = self.classes[np.argmax(ret)]
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('classify_sound_image_ros')
    csir = ClassifySoundImageROS()
    rospy.spin()
