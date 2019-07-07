Usage
=====

### Use sound spectrum directly

1. Save sound spectrum in `~/hitting_sound_data/spectrum`. Specify target object name (e.g. apple).
```bash
roslaunch hitting_sound_classification mini_microphone.launch save_spectrum:=true hitting_target:=(taget object name)
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

### Use sound image

1. Save sound image in `~/hitting_sound_data/image/origin`. Specify target object name (e.g. apple).
```bash
roslaunch hitting_sound_classification mini_microphone.launch save_image:=true hitting_target:=(taget object name)
```

1-2. Visualize saved images
```bash
python visualize_saved_image.py --classname (target object name)
```

2. Create dataset for training.
```bash
python create_dataset.py --path ~/hitting_sound_data/image
```

3. Train with chainer.
```bash
./train.py --gpu 0
```

4. Classify sound spectrum on ROS.
not yet

4.

5. Cross validation for collected sound image
not yet


Microphone
==========
MINI Microphone
http://akizukidenshi.com/catalog/g/gM-12864/
