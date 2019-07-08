Usage
=====

### Use sound image

1. Save sound image in `~/hitting_sound_data/image/origin`. Specify target object name (e.g. apple).
```bash
roslaunch hitting_sound_classification mini_microphone.launch save_image:=true target_class:=(taget object name)
```

2. Create dataset for training. (Train data is augmented, but test data is not augmented.)
```bash
python create_dataset.py
```

3. Visualize saved images (train or test must be selected as an argument)
```bash
python visualize_saved_image.py train
```

4. Train with chainer. Results are in `scripts/result`
```bash
./train.py --gpu 0 --epoch 100 --test
```
NOTE: Only `NIN` architecture is available now. (Other models are for ImageNet, 1000 class classification)

5. Classify sound image on ROS. (Results are Visualized in rqt)
```bash
roslaunch hitting_sound_classification mini_microphone.launch
```

6. Record/Play rosbag
```bash
# record
roslaunch hitting_sound_classification record_sound_image_classification.launch filename:=$HOME/hoge.bag
# play
roslaunch hitting_sound_classification play_sound_image_classification.launch filename:=$HOME/hoge.bag
```


### Use sound spectrum directly

1. Save sound spectrum in `~/hitting_sound_data/spectrum`. Specify target object name (e.g. apple).
```bash
roslaunch hitting_sound_classification mini_microphone.launch save_spectrum:=true target_class:=(taget object name)
```

2. Classify sound spectrum on ROS.
```bash
roslaunch hitting_sound_classification mini_microphone.launch
rostopic echo /object_class
```

3. Cross validation for collected sound spectrum
```bash
roscd hitting_sound_classification/node_scripts
python classify_hitting_sound.py
```

Microphone
==========
MINI Microphone
http://akizukidenshi.com/catalog/g/gM-12864/
