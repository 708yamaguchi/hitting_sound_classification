Usage
=====

1. Save sound data in `~/hitting_sound_data/`. Specify target object name (e.g. apple).
```bash
roslaunch hitting_sound_classification mini_microphone.launch save_spectrum:=true hitting_target:=(taget_object name)
```

2. Classify sound on ROS.
```bash
roslaunch hitting_sound_classification mini_microphone.launch
rostopic echo /object_class
```

3. Cross validation for collected sound data
```bash
roscd hitting_sound_classification/scripts
python classify_hitting_sound.py
```


Microphone
==========
MINI Microphone
http://akizukidenshi.com/catalog/g/gM-12864/
