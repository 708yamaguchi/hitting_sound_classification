#!/usr/bin/env python
# -*- coding: utf-8 -*-

# example usage
# ./visualize_saved_image.py -c table
# (visualize ~/hitting_sound_data/image/origin/table/**.png)

import cv2
import numpy as np
import os
import os.path as osp
import argparse

from PIL import Image as Image_


def visualize():
    parser = argparse.ArgumentParser()
    parser.add_argument('type', help='visualize train or test', choices=['train', 'test'])
    args = parser.parse_args()
    data_dir = osp.join(os.environ['HOME'], 'hitting_sound_data', 'image', 'dataset')
    for f in os.listdir(data_dir):
        if not f.startswith(args.type):
            continue
        img = np.array(Image_.open(osp.join(data_dir, f)))
        cv2.imshow('RGB labeled Image', img)
        print('{}'.format(f))
        key = cv2.waitKey(0)
        if key == ord('q'):
            exit()


if __name__ == '__main__':
    visualize()
