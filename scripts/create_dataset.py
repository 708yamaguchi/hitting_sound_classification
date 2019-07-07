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


import numpy as np
import imgaug as ia
import imgaug.augmenters as iaa


ia.seed(1)
seq = iaa.Sequential([
    # iaa.Fliplr(0.5), # horizontal flips
    # iaa.Crop(percent=(0, 0.1)), # random crops
    # Small gaussian blur with random sigma between 0 and 0.5.
    # But we only blur about 50% of all images.
    iaa.Sometimes(0.5,
        iaa.GaussianBlur(sigma=(0, 0.5))
    ),
    # Strengthen or weaken the contrast in each image.
    iaa.ContrastNormalization((0.75, 1.5)),
    # Add gaussian noise.
    # For 50% of all images, we sample the noise once per pixel.
    # For the other 50% of all images, we sample the noise per pixel AND
    # channel. This can change the color (not only brightness) of the
    # pixels.
    iaa.AdditiveGaussianNoise(loc=0, scale=(0.0, 0.05*255), per_channel=0.5),
    # Make some images brighter and some darker.
    # In 20% of all cases, we sample the multiplier once per channel,
    # which can end up changing the color of the images.
    iaa.Multiply((0.8, 1.2), per_channel=0.2),
    # Apply affine transformations to each image.
    # Scale/zoom them, translate/move them, rotate them and shear them.
    iaa.Affine(
        # scale={"x": (0.8, 1.2), "y": (0.8, 1.2)},
        scale={"x": (0.8, 1.2), "y": (1.0, 1.0)},
        # translate_percent={"x": (-0.2, 0.2), "y": (-0.2, 0.2)},
        translate_percent={"x": (-0.2, 0.2), "y": (0, 0)},
        # rotate=(-25, 25),
        # shear=(-8, 8)
    )
], random_order=True)  # apply augmenters in random order


def split():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--rate', default='0.8')  # train:test = 0.8:0.2
    parser.add_argument('-p', '--path', default=osp.expanduser('~/hitting_sound_data/image/'))
    parser.add_argument('-a', '--augment', default='5')  # create (augment) images per 1 image
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
        # resize and augment data (multiple args.augment times)
        for i, file_name in enumerate(file_names):
            saved_file_name = class_name + file_name
            img = Image_.open(osp.join(origin_dir, class_name, file_name))
            img_resize = img.resize((256, 256))
            if i < file_num * rate:  # save data for train
                saved_file_name = 'train_' + saved_file_name
                for j in range(int(args.augment)):
                    _ = osp.splitext(saved_file_name)
                    saved_file_name_augmented = _[0] + '_{0:03d}'.format(j) + _[1]
                    img_aug = Image_.fromarray(seq.augment_image(np.array(img_resize)))
                    img_aug.save(osp.join(dataset_dir, saved_file_name_augmented))
                    image_list_train.append(saved_file_name_augmented + ' ' + str(class_id) + '\n')
                    print('saved {}'.format(saved_file_name_augmented))
            else:  # save data for test
                saved_file_name = 'test_' + saved_file_name
                img_resize.save(osp.join(dataset_dir, saved_file_name))
                image_list_test.append(saved_file_name + ' ' + str(class_id) + '\n')
                print('saved {}'.format(saved_file_name))

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
