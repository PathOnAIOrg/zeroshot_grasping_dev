# LeRobot Sim2Real

LeRobot Sim2real provides code to train with Reinforcement Learning in fast GPU parallelized simulation and rendering via [ManiSkill](https://github.com/haosulab/ManiSkill) and deploy to the real-world. The codebase is designed for use with the [🤗 LeRobot](https://github.com/huggingface/lerobot) library, which handles all of the hardware interfacing code. Once you clone and follow the installation instructions you can try out the [zero-shot RGB sim2real tutorial](./docs/zero_shot_rgb_sim2real.md) to train in pure simulation something that can pick up cubes in the real world like below:

https://github.com/user-attachments/assets/ca20d10e-d722-48fe-94af-f57e0b2b2fcd

Note that this project is still in a very early stage. There are many ways the sim2real can be improved (like more system ID tools, better reward functions etc.), but we plan to keep this repo extremely simple for readability and hackability.

If you find this project useful, give this repo and [ManiSkill](https://github.com/haosulab/ManiSkill) a star! If you are using [SO100](https://github.com/TheRobotStudio/SO-ARM100/)/[LeRobot](https://github.com/huggingface/lerobot), make sure to also give them a star. If you use ManiSkill / this sim2real codebase in your research, please cite our [research paper](https://arxiv.org/abs/2410.00425):

```
@article{taomaniskill3,
  title={ManiSkill3: GPU Parallelized Robotics Simulation and Rendering for Generalizable Embodied AI},
  author={Stone Tao and Fanbo Xiang and Arth Shukla and Yuzhe Qin and Xander Hinrichsen and Xiaodi Yuan and Chen Bao and Xinsong Lin and Yulin Liu and Tse-kai Chan and Yuan Gao and Xuanlin Li and Tongzhou Mu and Nan Xiao and Arnav Gurha and Viswesh Nagaswamy Rajesh and Yong Woo Choi and Yen-Ru Chen and Zhiao Huang and Roberto Calandra and Rui Chen and Shan Luo and Hao Su},
  journal = {Robotics: Science and Systems},
  year={2025},
}
```

## 1. Getting Started

### 1.1 install maniskill

Install this repo by running the following
```bash
conda create -n ms3-lerobot "python==3.11" # 3.11 is recommended
conda activate ms3-lerobot
git clone https://github.com/StoneT2000/lerobot-sim2real.git
cd lerobot-sim2real 
pip install -e .
pip install torch # install the version of torch that works for you
```

The ManiSkill/SAPIEN simulator code is dependent on working NVIDIA drivers and vulkan packages. After running pip install above, if something is wrong with drivers/vulkan, please follow the troubleshooting guide here: https://maniskill.readthedocs.io/en/latest/user_guide/getting_started/installation.html#troubleshooting

To double check if the simulator is installed correctly, you can run 

```
python -m mani_skill.examples.demo_random_action

info {'elapsed_steps': tensor([48], dtype=torch.int32), 'success': tensor([False])}
reward tensor([0.0788])
terminated tensor([False])
truncated tensor([False])
info {'elapsed_steps': tensor([49], dtype=torch.int32), 'success': tensor([False])}
reward tensor([0.0788])
terminated tensor([False])
truncated tensor([True])
info {'elapsed_steps': tensor([50], dtype=torch.int32), 'success': tensor([False])}
Destroyed VkDevice on GPU Apple M3 Pro with 4 Vulkan extensions enabled.
```

test with hello world
```
python -m sapien.example.hello_world
```

test with push-T environment
```
python -m mani_skill.examples.demo_random_action -e PushT-v1 --render-mode="human"  
```

test with so100
```
python -m mani_skill.examples.demo_random_action -e PickCubeSO100-v1 --render-mode="human"
```


### 1.2 install lerobot
Then we install lerobot which enable ease of use with all kinds of hardware.

```bash
# Download the specific commit directly
wget https://github.com/huggingface/lerobot/archive/a989c795587d122299275c65a38ffdd0a804b8dc.zip -O lerobot.zip
unzip lerobot.zip
cd lerobot-a989c795587d122299275c65a38ffdd0a804b8dc
# note that the code was based on a slightly older lerobot commit. LeRobot recently changed the location of a few files we import so it broke some imports
# latest LeRobot can work but some LeRobot import paths need to be updated
pip install -e .
```

Note that depending on what hardware you are using you might need to install additional packages in LeRobot. If you already installed lerobot somewhere else you can use that instead of running the command above.

## Sim2Real Tutorial

We currently provide a tutorial on how to train a RGB based model controlling an SO100 robot arm in simulation and deploying that zero-shot in the real world to grasp cubes. Follow the tutorial [here](./docs/zero_shot_rgb_sim2real.md). Note while SO101 looks similar to SO100, we have found that there are some key differences that make sim2real fail for SO101, we will updaye this repository once SO101 is modelled correctly.

We are also working on a tutorial showing you how to make your own environments ready for sim2real, stay tuned!
