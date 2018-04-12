# aero_ssd_recognition #

**Requirements**
- CUDA
- OpenCV 3.1.0
- ROS (Kinetic or Indigo)
- chainer
- chainercv

----

## FCN Object Detector ##

Detect items by SSD and publish 3d positions of them.

### Pretrained models ###

To launch:

```
roslaunch aero_ssd_recognition fcn_object_detector.launch
```

To start detection:

```
rosservice call /object_detector/set_mode "data: true"
```

To select model:

```
rosservice call /object_detector/set_parameters "config:
  bools:
  - {name: '', value: false}
  ints:
  - {name: '', value: 0}
  strs:
  - {name: 'model_name', value: 'HOGEHOGE'}
  doubles:
  - {name: '', value: 0.0}
  groups:
  - {name: '', state: false, id: 0, parent: 0}"
```

The list of models is shown in  
aero\_object\_detector/models/chainer/model\_list.yaml (automatically generated)  
and the lists of items are shown in HOGE\_class\_list.txt in each model directory.

To change projection mode:

```
rosservice call /object_3d_projector/set_parameters "config:
  bools:
  - {name: '', value: false}
  ints:
  - {name: 'mode', value: 0~3}
  strs:
  - {name: '', value: ''}
  doubles:
  - {name: '', value: 0.0}
  groups:
  - {name: '', state: false, id: 0, parent: 0}"
```

prepared modes are:
- 0 : return the position of center of object
- 1 : return the position of center of object (slightly lower than 0)
- 2 : return the nearest point of object
- 3 : return the position which has the same direction as 0, same distance as 2.

### Training sample ###

You can use [train.py](scripts/chainer/train.py) for training with your original dataset.  
Sample of command to run training is:

```
roscd aero_ssd_recognition/scripts/chainer/
./train.py --data /PATH_TO_DATA/train_data/ --class_list /PATH_TO_DATA/train_data/class_list.txt --step_size 10000 --batchsize 20 --val_iteration 2000 --out result/face-result
```

See ```./train.py --help``` for the other options.

Terminate script after the training converged, then copy trained model as below.

```
cp result/sample/model_iter_XXXX ../../models/chainer/YYYY/YYYY_model.npz
cp ../../data/sample/final_demo/class_list ../../models/chainer/YYYY/YYYY_class_list.txt
```

---

## Dummy Nodes ##

You can use dummy mode detection by compiling this package with ```-D DUMMY=ON``` option and
launch the above launches with ```dummy:=true``` option.
