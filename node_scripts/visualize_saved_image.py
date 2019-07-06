#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import os
import os.path as osp
import argparse


def visualize():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--classname', default='table')
    args = parser.parse_args()
    data_dir = osp.join(os.environ['HOME'], 'hitting_sound_data', 'image', args.classname)
    for f in os.listdir(data_dir):
        img = np.load(osp.join(data_dir, f))
        cv2.imshow('RGB labeled Image', img)
        key = cv2.waitKey(0)
        if key == ord('q'):
            exit()


if __name__ == '__main__':
    visualize()
