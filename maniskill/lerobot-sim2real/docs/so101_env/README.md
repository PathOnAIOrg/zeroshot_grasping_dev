# Maniskill SO101 Environment

## SO101 Custom Agent 
├── so101_env/
│ ├── so101_arm.py              # defines MySO101 agent
│ └── SO101/
│ └── so101_new_calib.urdf      # URDF for the robot arm
├── testso101loading.py                 # Script to test the so101 loading in sapien

so101_env/
├── so101_arm.py                # defines the MySO101 agent class
├── SO101/
│ └── so101_new_calib.urdf      # URDF model for the SO101 robot
├── testso101loading.py         # script to test loading the SO101 robot in SAPIEN
├── PickCubeSO101.py            # registers a PickCube task for SO101
└── test_so101env.py            # runs the PickCubeSO101-v1 environment with rendering

Run the following to open the simulation visualizer (without taking any actions) to let you look at the robot:  

```
python maniskill/lerobot-sim2real/docs/so101_env/testso101loading.py -r "my_so101
```


Run the following to launch PickCube environment with redering using SO101 robot

```
python maniskill/lerobot-sim2real/docs/so101_env/test_so101env.py
```