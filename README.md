Usage
=====

## Quick demo
Type Commands below:
```
rosrun hitting_sound_classification create_dataset.py            # create dataset
rosrun hitting_sound_classification train.py --gpu 0 --epoch 100 # train
roslaunch hitting_sound_classification microphone.launch         # classification on ROS
```

## Commands

1. Save spectrogram in `train_data/original_spectrogram`. Specify target object name (e.g. apple).
```bash
roslaunch hitting_sound_classification microphone.launch save_image:=true target_class:=(taget object name)
```

2. Create dataset for train and test. (Train data is augmented, but test data is not augmented.)
```bash
rosrun hitting_sound_classification create_dataset.py
```

3. Visualize created dataset (train or test must be selected as an argument)
```bash
rosrun hitting_sound_classification visualize_dataset.py train
```

4. Train with chainer. Results are in `scripts/result`
```bash
rosrun hitting_sound_classification train.py --gpu 0 --epoch 100
```
NOTE: Only `NIN` architecture is available now.

5. Classify spectrogram on ROS. (Results are visualized in rqt)
```bash
roslaunch hitting_sound_classification microphone.launch
```

6. Record/Play rosbag
```bash
# record
roslaunch hitting_sound_classification record_sound_image_classification.launch filename:=$HOME/hoge.bag
# play
roslaunch hitting_sound_classification play_sound_image_classification.launch filename:=$HOME/hoge.bag
```

Experiment
==========
3 class classification using spectrogram (applause, flick, voice)
![Experiment](https://github.com/708yamaguchi/hitting_sound_classification/blob/media/spectrogram_classification_with_thinkpad.gif)


Upper left : Estimated class
Left       : spectrogram
Right      : Video


Microphone
==========
ThinkPad T460s build-in microphone
