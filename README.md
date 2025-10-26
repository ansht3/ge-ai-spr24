# VIP Autonomous Robotics (GE AI) - Spring 2024

## ❓ Introduction

This was a project that a group of 5 members did as a part of a research class over the Spring 2024 semester, in collaboration with GE Research industry partner.

## ⚙️ Technologies Used
- Python
- PyTorch
- PyBullet
- Hugging Face (LeRobot)
- OpenAI Gymnasium
- Gazebo3D
- XML
- RoboHive

### Supported Platforms
- Ubuntu
- Windows
    - Must use [WSL](https://docs.microsoft.com/en-us/windows/wsl) (Windows Subsystem for Linux)
- MacOS

## Part 1 of Project

Please take the GE_Research_Project_Part_1.ipynb and run each cell as outlined in the file. Ensure you are utilizing Python >=3.10.0 to run the file and ensure all dependencies can be properly installed.

## Part 2 of Project


### Preliminary Steps
1. Install MuJoCo and pyenv (python version management)
- [Install Mujoco](https://mujoco.readthedocs.io/en/stable/programming/index.html#getting-started)
- Install pyenv

Linux/WSL:
```
sudo apt update
sudo apt-get install -y build-essential zlib1g-dev libffi-dev libssl-dev liblzma-dev libbz2-dev libreadline-dev libsqlite3-dev
curl https://pyenv.run | bash
echo -e 'export PATH="$HOME/.pyenv/bin:$PATH"\neval "$(pyenv init --path)"\neval "$(pyenv init -)"\neval "$(pyenv virtualenv-init -)"' >> ~/.bashrc
source ~/.bashrc
```
> NOTE: echo command adds pyenv to .bashrc to start it every time you open a terminal

MacOS:
```
brew install pyenv
brew install pyenv-virtualenv
```
2. Install known working version of python
```
pyenv install 3.9.18
```

3. Clone repo 

```
git clone https://github.com/purdue-mars/VIP-GEAI
cd 02-block-pick-and-place
```

4. Create virtual environemnt, install repo, and dependencies 

```
pyenv virtualenv 3.9.18 pick_n_place 
pyenv activate pick_n_place
pip3 install -e .
```

#### Part 1: Open-loop Box Pick and Place

Goal: get the robot to pick up the brick in the right bin and place it in the left bin within the **target green zone** via following a precomputed trajectory open-loop.

Below is a video of our demo:



https://github.com/user-attachments/assets/d09170df-1473-4a1d-a013-b5837c27d1e0



To run the starter script:
``` 
cd pick_n_place
python3 part_1/main.py 
```
> NOTE: Initially, the robot will just go to the **target green zone**

Update the `part_1/main.py` file with the following steps:
1. STEP 1.1: Generate Task-relevant Poses
2. STEP 1.2: Create Joint-position trajectory following the task-relevant poses

Helpful tips:

- On the left sidebar of the MuJoCo Renderer, click: 
  - `Rendering > Frame > Site|Body|Geom` to toggle rendering of the coordinate frames
  - `Rendering > Label > Site|Body|Geom` to toggle rendering of the labels
- Helpful utils are provided for you to in the robohive library:
  - [quat2mat](https://github.com/vikashplus/robohive/blob/ef6f2c3deb93555d779bb3f9af0b3c21414c6bc0/robohive/utils/quat_math.py#L152): convert quaternion to rotation matrix
  - [mat2quat](https://github.com/vikashplus/robohive/blob/ef6f2c3deb93555d779bb3f9af0b3c21414c6bc0/robohive/utils/quat_math.py#L110): convert rotation matrix to quaternion
  - [generate_joint_space_min_jerk](https://github.com/vikashplus/robohive/blob/ef6f2c3deb93555d779bb3f9af0b3c21414c6bc0/robohive/utils/min_jerk.py#L5): generate joint space trajectory given start and end joint angles

#### Part 2: Box Pick and Place with Collision Avoidance

Utilizing motion planning algorithms to create collision-free trajectories to complete the pick and place task, with Hugging Face LeRobot library.

Below is a video of our training our model utilizing LeRobot:



https://github.com/user-attachments/assets/ec677e48-79f3-45e3-89bd-6d1330d1d7ac



#### Part 3: Closed-loop Box Pick and Place using Vision and Learning

Often, a robotic system may not have perfect perception of the object pose in the real world, how can the arm still perform robust pick and place using only raw images as input?

Get an introduction to learning techniques commonly found in robotics:
- reinforcement learning
- behavior cloning

We will go ahead and implement this in future iterations of this project.

## 🫂 Acknowledgements
- Thank you Shaopeng Liu @ GE Research for meeting with us to go over our work throughout the semester!
