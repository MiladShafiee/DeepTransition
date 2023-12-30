
# Deep Transition Repository #
 :tiger2:  This repository provides an implementation of the gap-crossing scenario in the following paper:
- Viability Leads to the Emergence of Gait Transitions in Learning Agile Quadrupedal Locomotion on Challenging Terrains
Milad Shafiee, Guillaume Bellegarda, Auke Ijspeert



## :hammer:  Installation (GPU and CUDA needed) ##
:floppy_disk:   Create a new python virtual env with python 3.6, 3.7 or 3.8 (3.8 recommended)
   
   - `virtualenv -p python3 DeepTransitionENV`
   - `source {PATH_TO_VENV}/DeepTransitionENV/bin/activate`
     
:page_facing_up: Run git clone on this repository and install dependencies:

- `pip install -r requirements.txt`

   
-  Install pytorch 1.10 with cuda-11.3 or (pytorch 1.11 and cuda-11.4) (tested with GPU RTX-3070 on Ubuntu 18.04 and Ubuntu 20.04)
   - `pip3 install torch==1.10.0+cu113 torchvision==0.11.1+cu113 torchaudio==0.10.0+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html`
   
   or alternatively, Install pytorch 1.13 with cuda-11.7 (tested with GPU RTX-4090 on Ubuntu 20.04)   
   - `pip install torch==1.13.0+cu117 torchvision==0.14.0+cu117 torchaudio==0.13.0 --extra-index-url https://download.pytorch.org/whl/cu117`
   
:cartwheeling:  Install Isaac Gym
   - Download and install Isaac Gym Preview 3   [Download](https://drive.switch.ch/index.php/s/czLtPhwuwunFCW5)
   - `cd isaacgym/python && pip install -e .`
   
:chart_with_upwards_trend:  Install rsl_rl (PPO)
   -  `cd rsl_rl  && pip install -e .` 
   
:mechanical_leg:  Install legged_gym
   -  `cd legged_gym && pip install -e .`

## :school:  CODE STRUCTURE  & :clapper:  Usage  ##
The training environment is defined by an env file (`quadruped.py`) and a config file (`quadruped_config.py`) that these classes use inheritance.  Central Pattern Generator (CPG) is defined by `CPG.py`.

:weight_lifting:  **Training**:  
- Isaac Gym requires an NVIDIA GPU. To train in the default configuration, we recommend a GPU with at least 8GB memory. The code can run on a smaller GPU if you decrease the number of parallel environments (`Cfg.env.num_envs`) or reducing the precision of  terrain meshes. Training will be slower with fewer environments. An example for training:
 
  `python legged_gym/scripts/train.py --task=quadruped  --max_iterations=3000 --num_envs=4096`
  
 - To run on CPU add following arguments: `--sim_device=cpu`, `--rl_device=cpu` (sim on CPU and rl on GPU is possible).
    
 -  To run without no rendering add `--headless`.
    
 -  To improve performance, once the training starts (and if your are not running headless) make sure to press `v` to stop the rendering. You can then enable it later to check the progress.
    
   - The trained policy is saved in `issacgym_anymal/logs/<experiment_name>/<date_time>_<run_name>/model_<iteration>.pt`. Where `<experiment_name>` and `<run_name>` are defined in the train config.
    
   -  The following command line arguments override the values set in the config files:
    
   - `--task` TASK: Task name.
     
   - `--num_envs` NUM_ENVS:  Number of environments to create.
     
   - `--max_iterations` MAX_ITERATIONS:  Maximum number of training iterations.
     
   :runner: Play a pre-trained policy:  
 
   - `python legged_gym/scripts/play.py --task=quadruped`

   - By default, the loaded policy is the last model of the last run of the experiment folder.


```
This environment builds on the [legged gym environment](https://leggedrobotics.github.io/legged_gym/) by Nikita
Rudin, Robotic Systems Lab, ETH Zurich (Paper: https://arxiv.org/abs/2109.11978) and the Isaac Gym simulator from 
NVIDIA (Paper: https://arxiv.org/abs/2108.10470). Training code builds on the [rsl_rl](https://github.com/leggedrobotics/rsl_rl) 
repository, also by Nikita Rudin, Robotic Systems Lab, ETH Zurich. 
All redistributed code retains its original [license](LICENSES/legged_gym/LICENSE).
```

