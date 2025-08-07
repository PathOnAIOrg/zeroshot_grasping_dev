### 7/23
mujoco so101 and blue cube model do not contact the ground.   
Investigation: stl files may need to recenter.

### 7/28
successfully calibrate so101 arms using the [official tutorial](https://huggingface.co/docs/lerobot/en/so101) and run teleoperate simulation.    
Note: it might be less intuitive for novice users to calibrate the leader arm, since the calibration video only covers the follower arm. we should consider that when we write tech blogs/tutorials.

### 7/30
able to run so100 simulation in the [lerobot-sim2real](https://github.com/StoneT2000/lerobot-sim2real) repo.   
add so101 model to the simulation.
Next step: double check physical properties, import custom objects.

### 7/31
able to run [bi-lerobot](https://github.com/LiZhYun/BiLerobot) pipeline [[video]](https://www.loom.com/share/0f1a9cf9a4fa4e56b2a423b5520293a6)
