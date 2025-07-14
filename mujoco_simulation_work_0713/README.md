# SO-ARM100 Simulation

## 2. MuJoCo 
```
python3.11 -m venv venv
. venv/bin/activate
```

### 2.1 test cube stacking gym environment
Here we added two cubes in the scene, and plan to implement the cube stacking environment
```
cd cube_stacking
mjpython cube_stacking_env.py
```
but the physical properties of the cubes seem off and the success definition of the gym environment is missing

### 2.2 add camera in mujoco and test
```
mjpython test_gym.py
```

This generates various camera views and saves them to the `images_front` directory:

![Top-down camera view](images_front/0000_topdown_image.png)
![Wrist camera view](images_front/0000_wrist_image.png)

The simulation captures multiple camera perspectives including top-down views, wrist camera views. The wrist camera view is wrong and the position need to be fixed.