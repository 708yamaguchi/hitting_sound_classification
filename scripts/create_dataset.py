#!/usr/bin/env python
# -*- coding: utf-8 -*-

# directory composition
# origin - class1 -- img001.png
#        |        |- img002.png
#        |        |- ...
#        - class2 -- img001.png
#                 |- img002.png
#                 |- ...
#
# -> (./create_dataset.py)
#
# origin - class1 -- img001.png
#        |        |- img002.png
#        |        |- ...
#        - class2 -- img001.png
#                 |- img002.png
#                 |- ...
# train  - class1 -- img001.png
#        |        |- ...
#        - class2 -- img001.png
#                 |- ...
# test   - class1 -- img002.png
#        |        |- ...
#        - class2 -- img002.png
#                 |- ...

import os
import os.path as osp
import argparse
import shutil


def split():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--rate', default='0.8')  # train:test = 0.8:0.2
    parser.add_argument('-p', '--path', default='~')
    args = parser.parse_args()
    rate = float(args.rate)
    root_dir = osp.expanduser(args.path)
    origin_dir = osp.join(root_dir, 'origin')
    train_dir = osp.join(root_dir, 'train')
    test_dir = osp.join(root_dir, 'test')
    # remove train and test dir if exist
    if osp.exists(train_dir):
        shutil.rmtree(train_dir)
    if osp.exists(test_dir):
        shutil.rmtree(test_dir)
    for class_name in os.listdir(origin_dir):
        file_names = os.listdir(osp.join(origin_dir, class_name))
        file_num = len(file_names)
        class_dir_in_train = osp.join(osp.split(origin_dir)[0], 'train', class_name)
        class_dir_in_test = osp.join(osp.split(origin_dir)[0], 'test', class_name)
        if not osp.exists(class_dir_in_train):
            os.makedirs(class_dir_in_train)
        if not osp.exists(class_dir_in_test):
            os.makedirs(class_dir_in_test)
        # copy train and test data
        for i, file_name in enumerate(file_names):
            if i < file_num * rate:
                shutil.copyfile(
                    osp.join(origin_dir, class_name, file_name),
                    osp.join(class_dir_in_train, file_name))
            else:
                shutil.copyfile(
                    osp.join(origin_dir, class_name, file_name),
                    osp.join(class_dir_in_test, file_name))


if __name__ == '__main__':
    split()
