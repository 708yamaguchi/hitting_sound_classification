Usage
=====

## Sound spectrogram classification

### Commands

1. Save spectrogram in `hitting_sound_data/image/origin`. Specify target object name (e.g. apple).
```bash
roslaunch hitting_sound_classification microphone.launch save_image:=true target_class:=(taget object name)
```

2. Create dataset for training. (Train data is augmented, but test data is not augmented.)
```bash
rosrun hitting_sound_classification create_dataset.py
```

3. Visualize saved images (train or test must be selected as an argument)
```bash
rosrun hitting_sound_classification visualize_dataset.py train
```

4. Train with chainer. Results are in `scripts/result`
```bash
rosrun hitting_sound_classification train.py --gpu 0 --epoch 100
```
NOTE: Only `NIN` architecture is available now. (Other models are for ImageNet, 1000 class classification)

5. Classify spectrogram on ROS. (Results are Visualized in rqt)
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

### Experiment
3 class classification using spectrogram (bed, table, tissue_box)
![Experiment](https://github.com/708yamaguchi/hitting_sound_classification/blob/media/sound_image_classification.gif)
Upper left : Estimated class
Left       : spectrogram
Right      : Video


Microphone
==========
ThinkPad T460s build-in microphone
