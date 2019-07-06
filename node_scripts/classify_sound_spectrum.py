#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import os
import os.path as osp

from sklearn.metrics import accuracy_score
from sklearn.neighbors import KNeighborsClassifier


class ClassifySoundSpectrum(object):

    def __init__(self):
        # load data
        self.data_dir = osp.join(os.environ['HOME'], 'hitting_sound_data', 'spectrum')
        self.data_for_class = {}
        self.class_names = os.listdir(self.data_dir)
        print('classification target classes:')
        for class_name in self.class_names:
            print(class_name)
            self.data_for_class[class_name] = None
            class_dir = osp.join(self.data_dir, class_name)
            for sound_data_file in os.listdir(class_dir):
                sound_data = np.load(osp.join(class_dir, sound_data_file))
                if self.data_for_class[class_name] is None:
                    self.data_for_class[class_name] = sound_data[None]
                else:
                    self.data_for_class[class_name] = np.append(
                        self.data_for_class[class_name], sound_data[None], axis=0)

    def cross_varidation(self, k=10, divide=5):
        # divide dataset for cross validation
        train_data_list = []
        train_labels_list = []
        test_data_list = []
        test_labels_list = []
        scores = np.array([], dtype=np.float32)
        for j in range(divide):
            for i, class_name in enumerate(self.class_names):
                sound_data = self.data_for_class[class_name]
                data_length = sound_data.shape[0]
                labels = np.full(data_length, i)
                # split train and test data
                test_start_index = int(j * data_length / float(divide))
                test_end_index = int((j + 1) * data_length / float(divide))
                test_index_array = np.full(data_length, False, dtype=np.bool)
                test_index_array[test_start_index:test_end_index] = True
                train_data_list.extend(sound_data[~test_index_array])
                train_labels_list.extend(labels[~test_index_array])
                test_data_list.extend(sound_data[test_index_array])
                test_labels_list.extend(labels[test_index_array])
            # load model of k nearest neighbor
            model = KNeighborsClassifier(n_neighbors=k)
            model.fit(train_data_list, train_labels_list)
            # predict
            predicted_label = model.predict(test_data_list)
            # calc score
            score = accuracy_score(test_labels_list, predicted_label)
            scores = np.append(scores, score)
        print("mean accuracy: {}".format(score))

    def load_data(self, k=10):
        # divide dataset for cross validation
        data_list = []
        labels_list = []
        for i, class_name in enumerate(self.class_names):
            sound_data = self.data_for_class[class_name]
            data_length = sound_data.shape[0]
            labels = np.full(data_length, i)
            # split train and test data
            data_list.extend(sound_data)
            labels_list.extend(labels)
        # load model of k nearest neighbor
        self.model = KNeighborsClassifier(n_neighbors=k)
        self.model.fit(data_list, labels_list)

    def predict(self, sound_data):
        return self.class_names[int(self.model.predict(sound_data)[0])]


if __name__ == '__main__':
    css = ClassifySoundSpectrum()
    css.cross_varidation(k=7)
