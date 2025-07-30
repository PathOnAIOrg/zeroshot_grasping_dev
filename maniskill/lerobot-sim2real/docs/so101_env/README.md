# Maniskill SO101 Environment

## SO101 Custom Agent 
├── so101_env/
│ ├── so101_arm.py              # defines MySO101 agent
│ └── SO101/
│ └── so101_new_calib.urdf      # URDF for the robot arm
├── testso101loading.py                 # Script to test the so101 loading in sapien

Run the following to open the simulation visualizer (without taking any actions) to let you look at the robot:  

```
python maniskill/lerobot-sim2real/docs/so101_env/testso101loading.py -r "my_so101
```
