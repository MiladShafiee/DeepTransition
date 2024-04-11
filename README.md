




# Deep Transition Repository #
 :tiger2: This repository provides an implementation (Simulation code) of the CPG-RL framework in the following paper in **Nature Communications**:
<td style="padding:20px;width:75%;vertical-align:middle">
      <a href="https://miladshafiee.github.io/DeepTransition/">
      <b> Viability Leads to the Emergence of Gait Transitions in Learning Agile Quadrupedal Locomotion on Challenging Terrains </b>
      </a>
      <br>
      <a href="https://miladshafiee.github.io/" target="_blank">Milad Shafiee</a> , <a href="https://people.epfl.ch/guillaume.bellegarda/" target="_blank">Guillaume Bellegarda</a> and
      <a href="https://www.epfl.ch/labs/biorob/people/ijspeert/" target="_blank">Auke Ijspeert</a>
      <br>
      <em>Nature Communications, vol. 15, no. 1, p. 3073</em>, 2024,.
      <br>
      <a href="https://doi.org/10.1038/s41467-024-47443-w">paper</a> /
      <a href="https://miladshafiee.github.io/DeepTransition/" target="_blank">project page</a>
    <br>
</td>


```
@article{shafiee2024Viability,
  title={Viability leads to the emergence of gait transitions in learning agile quadrupedal locomotion on challenging terrains},
  author={Shafiee, Milad and Bellegarda, Guillaume and Ijspeert, Auke },
  journal={Nature Communications},
  volume={15},
  number={1},
  pages={3073},
  year={2024},
  publisher={Nature Publishing Group UK London}
}
```
<p align="center">
  <img src="images/exp.gif" width="100%"/>
</p>

##  :hammer: System requirements and Installation (GPU > 8GB and CUDA needed) ##
 We need GPU and Cuda installed for running the Isaac Gym simulator. To train in the default configuration, we recommend a GPU with at least 8GB memory.
  The code tested with GPU RTX-3070 (cuda-11.4) and  GPU RTX-4090 (cuda-11.7). Installation of Nvidia drivers and Cuda may take couple of hours depending on the hardware.
  After having cuda installed and GPU ready, installation ideally will take less than 15 minutes:

:floppy_disk: Create a new python virtual env with python 3.6, 3.7 or 3.8 (3.8 recommended)
   
   - `virtualenv -p python3 DeepTransitionENV`
   - `source DeepTransitionENV/bin/activate`
     
:page_facing_up:  Clone this repository and install dependencies:

- `pip install -r requirements.txt`

   
-  Install pytorch 1.10 with cuda-11.3 or (pytorch 1.11 and cuda-11.4) (tested with GPU RTX-3070 on Ubuntu 18.04 and Ubuntu 20.04)
   - `pip3 install torch==1.10.0+cu113 torchvision==0.11.1+cu113 torchaudio==0.10.0+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html`
   
   or alternatively, Install pytorch 1.13 with cuda-11.7 (tested with GPU RTX-4090 on Ubuntu 20.04)   
   - `pip install torch==1.13.0+cu117 torchvision==0.14.0+cu117 torchaudio==0.13.0 --extra-index-url https://download.pytorch.org/whl/cu117`
   
:cartwheeling:  Install Isaac Gym
   - Download and install Isaac Gym Preview 3   [Download](https://developer.nvidia.com/isaac-gym)
   - `cd isaacgym/python && pip install -e .`
   - Check installation by running an example: `cd examples && python 1080_balls_of_solitude.py`
   
:chart_with_upwards_trend:  Install rsl_rl (PPO)
   -  `cd rsl_rl  && pip install -e .` 
   
:mechanical_leg:  Install legged_gym
   -  `cd legged_gym && pip install -e .`

## :school: CODE STRUCTURE  ##
The training environment is defined by an env file (`quadruped.py`) and a config file (`quadruped_config.py`) that these classes use inheritance. quadruped_config.py includes body names, default_joint_positions and PD gains, reward weights,etc. You need to modify the reward weights in quadruped_config.py and train the policy to reproduce the result of the paper.
The config file contains two classes: one containing all the environment parameters (LeggedRobotCfg) and one for the training parameters (LeggedRobotCfgPPo).
 Central Pattern Generator (CPG) is defined by `CPG.py`.
 
## :weight_lifting:  Instructions for training  ##

- Isaac Gym requires an NVIDIA GPU. To train in the default configuration, we recommend a GPU with at least 8GB memory. The code can run on a smaller GPU if you decrease the number of parallel environments (`Cfg.env.num_envs`) or reducing the precision of  terrain meshes. Training will be slower with fewer environments.
 An example for training (training will take less than two hours with the suggest GPUs):
 
  `python legged_gym/scripts/train.py --task=quadruped  --max_iterations=3000 --num_envs=4096`
  
 - To run on CPU add following arguments: `--sim_device=cpu`, `--rl_device=cpu` (sim on CPU and rl on GPU is possible). Running on CPU is not encouraged and may lead to different results.
    
 -  To run without no rendering add `--headless`.
    
 -  To improve performance, once the training starts (and if your are not running headless) make sure to press `v` to stop the rendering. You can then enable it later to check the progress.
    
   - The trained policy is saved in `issacgym_anymal/logs/<experiment_name>/<date_time>_<run_name>/model_<iteration>.pt`. Where `<experiment_name>` and `<run_name>` are defined in the train config.
    
   -  The following command line arguments override the values set in the config files:
    
   - `--task` TASK: Task name.
     
   - `--num_envs` NUM_ENVS:  Number of environments to create.
     
   - `--max_iterations` MAX_ITERATIONS:  Maximum number of training iterations.
   
## :runner: Play a pre-trained demo  ##
     
 Play a pre-trained policy (It will take less than one minute):
 
   - `python legged_gym/scripts/play.py --task=quadruped`

   - By default, the loaded policy is the last model of the last run of the experiment folder.


## Acknowledgement  ##
```
This environment builds on the amazing repository [legged gym environment](https://leggedrobotics.github.io/legged_gym/) by Nikita
Rudin, Robotic Systems Lab, ETH Zurich (Paper: https://arxiv.org/abs/2109.11978) and the Isaac Gym simulator from 
NVIDIA (Paper: https://arxiv.org/abs/2108.10470). Training code builds on the [rsl_rl](https://github.com/leggedrobotics/rsl_rl) 
repository, also by Nikita Rudin, Robotic Systems Lab, ETH Zurich. 
All redistributed code retains its original [license](LICENSES/legged_gym/LICENSE).
```

