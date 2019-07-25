Usage
=====

## Quick demo
This is sound classification demo using ThinkPad's build-in camera and microphone.
```
rosrun hitting_sound_classification create_dataset.py            # create dataset
rosrun hitting_sound_classification train.py --gpu 0 --epoch 100 # train
roslaunch hitting_sound_classification microphone.launch         # classification on ROS
```

## Commands

1. Save your original spectrogram in `train_data/original_spectrogram`. Specify target object class.
```bash
roslaunch hitting_sound_classification microphone.launch save_image:=true target_class:=(taget object class)
```
NOTE: You can change microphone by giving `microphone_name` argument to this roslaunch. The names of microphones can be seen by `pyaudio.PyAudio().get_device_info_by_index(index)` fuction.
NOTE: You can change threshold of hitting detection by giving `hit_volume_threshold` argument to this roslaunch.

2. Create dataset for training with chainer. (Train dataset is augmented, but test dataset is not augmented.)
```bash
rosrun hitting_sound_classification create_dataset.py
```

3. Visualize created dataset (`train` or `test` must be selected as an argument)
```bash
rosrun hitting_sound_classification visualize_dataset.py train
```

4. Train with chainer. Results are output in `scripts/result`
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
