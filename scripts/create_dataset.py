#!/usr/bin/env python

# -*- coding: utf-8 -*-

# directory composition
# origin - classA -- 001.png
#        |        |- 002.png
#        |        |- ...
#        - classB -- 001.png
#                 |- 002.png
#                 |- ...
#
# -> (./create_dataset.py)
#
# origin - classA -- 001.png
#        |        |- 002.png
#        |        |- ...
#        - classB -- 001.png
#                 |- 002.png
#                 |- ...
# train  -- images.txt
#        |- classA001.png
#        |- classB001.png
#        |- ...
# train  -- images.txt
#        |- classA002.png
#        |- classB002.png
#        |- ...


import os
import os.path as osp
import argparse
import shutil

from PIL import Image as Image_


def split():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--rate', default='0.8')  # train:test = 0.8:0.2
    parser.add_argument('-p', '--path', default='~')
    args = parser.parse_args()
    rate = float(args.rate)
    root_dir = osp.expanduser(args.path)
    origin_dir = osp.join(root_dir, 'origin')
    # train_dir = osp.join(root_dir, 'train')
    # test_dir = osp.join(root_dir, 'test')
    dataset_dir = osp.join(root_dir, 'dataset')
    image_list_train = []
    image_list_test = []
    # remove train and test dir if exist
    # if osp.exists(train_dir):
    #     shutil.rmtree(train_dir)
    # os.mkdir(train_dir)
    # if osp.exists(test_dir):
    #     shutil.rmtree(test_dir)
    # os.mkdir(test_dir)
    if osp.exists(dataset_dir):
        shutil.rmtree(dataset_dir)
    os.mkdir(dataset_dir)
    for class_id, class_name in enumerate(os.listdir(origin_dir)):
        file_names = os.listdir(osp.join(origin_dir, class_name))
        file_num = len(file_names)
        # class_dir_in_train = osp.join(osp.split(origin_dir)[0], 'train', class_name)
        # class_dir_in_test = osp.join(osp.split(origin_dir)[0], 'test', class_name)
        # if not osp.exists(class_dir_in_train):
        #     os.makedirs(class_dir_in_train)
        # if not osp.exists(class_dir_in_test):
        #     os.makedirs(class_dir_in_test)

        # copy train and test data
        for i, file_name in enumerate(file_names):
            saved_file_name = class_name + file_name
            img = Image_.open(osp.join(origin_dir, class_name, file_name))
            img_resize = img.resize((256, 256))
            img_resize.save(osp.join(dataset_dir, saved_file_name))
            # shutil.copyfile(
            #     osp.join(origin_dir, class_name, file_name),
            #     osp.join(dataset_dir, saved_file_name))
            if i < file_num * rate:
                image_list_train.append(saved_file_name + ' ' + str(class_id) + '\n')
            else:
                image_list_test.append(saved_file_name + ' ' + str(class_id) + '\n')

        # create images.txt
        # for train
        file_path = osp.join(dataset_dir, 'train_images.txt')
        with open(file_path, mode='w') as f:
            for line_ in image_list_train:
                f.write(line_)
        # for test
        file_path = osp.join(dataset_dir, 'test_images.txt')
        with open(file_path, mode='w') as f:
            for line_ in image_list_test:
                f.write(line_)


if __name__ == '__main__':
    split()
