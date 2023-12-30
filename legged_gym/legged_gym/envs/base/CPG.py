# SPDX-FileCopyrightText: Copyright (c) 2023 EPFL Biorobotics Laboratory (BioRob). All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2023 EPFL Biorobotics Laboratory (BioRob)

from time import time
from warnings import WarningMessage
import numpy as np
import os
import torch

class CPG_RL():
    """ CPG-RL Implementation. 
    IsaacGym order is FL, FR, RL, RR (alphabetical)
    """
    LEG_INDICES = np.array([1,0,3,2])
    def __init__(self,
          omega_swing=8*2*np.pi,
          omega_stance=2*2*np.pi, 
          gait="TROT",
          couple=False,
          coupling_strength=1,
          time_step=0.001,
          robot_height=0.3, 
          des_step_len=0.03, 
          ground_clearance=0.07,  
          ground_penetration=0.01,
          num_envs=1,
          device=None,
          rl_task_string=None,
          mu_low = 1.0,
          mu_up = 4.0,
          max_step_len = 0.03,
        ):
        self._rl_task_string = rl_task_string
        #global device
        self._device = device 
        self.X = torch.zeros(num_envs,2,4,dtype=torch.float, device=device, requires_grad=False)
        self.X_dot = torch.zeros(num_envs,2,4,dtype=torch.float, device=device, requires_grad=False)
        self.d2X = torch.zeros(num_envs,1,4,dtype=torch.float, device=device, requires_grad=False)
        self.num_envs = num_envs
        self._mu = torch.zeros(num_envs,4,dtype=torch.float, device=device, requires_grad=False)
        if "OFFSETX" in rl_task_string:
            self._offset_x= torch.zeros(num_envs,4,dtype=torch.float, device=device, requires_grad=False)
            self._offset_z = torch.zeros(num_envs,4,dtype=torch.float, device=device, requires_grad=False)
        self.y = torch.zeros(num_envs,4,dtype=torch.float, device=device, requires_grad=False)
        self.mu_low= mu_low,
        self.mu_up = mu_up,
        self.max_step_len = max_step_len,
        self._omega_swing = omega_swing
        self._omega_stance = omega_stance  
        self._couple = couple
        self._coupling_strength = coupling_strength
        self._dt = time_step
        self._set_gait(gait)
 
        self.X[:,0,:] = torch.rand(num_envs,4,device=self._device) * .1
        self.X[:,1,:] = self.PHI[0,:] #*0.0

        self._ground_clearance = ground_clearance 
        self._ground_penetration = ground_penetration
        self._robot_height = robot_height 
        self._des_step_len = des_step_len

    def reset(self,env_ids):
        self._mu[env_ids,:] = 0
        self.X[env_ids,0,:] = torch.rand(len(env_ids),4,device=self._device) * .1
        self.X[env_ids,1,:] = self.PHI[0,:] #*0.0
        self.X_dot[env_ids,:,:] = 0.
 

    def _set_gait(self,gait):
        device = self._device 

        trot = torch.tensor([[ 0, 1, 1, 0 ],
                            [-1, 0, 0,-1 ],
                            [-1, 0, 0,-1 ],
                            [ 0, 1, 1, 0 ]],dtype=torch.float, device=device, requires_grad=False) 
        trot = np.pi * trot
        self.PHI_trot = trot

        if gait == "TROT":
            print('TROT')
            self.PHI = self.PHI_trot
        else:
            raise ValueError( gait + 'not implemented.')


    def update_offset_x(self,offset):
        self._offset_x = offset


    def update(self):
        """ Update oscillator states. """
        device = self._device 

        # update parameters, integrate
        self._integrate_hopf_equations()

        # map CPG variables to Cartesian foot xz positions
        x = self.X[:,0,:] * torch.cos(self.X[:,1,:]) 
        z = torch.where(torch.sin(self.X[:,1,:]) > 0, 
                        -self._robot_height + self._ground_clearance*torch.sin(self.X[:,1,:]),# swing)
                        -self._robot_height + self._ground_penetration*torch.sin(self.X[:,1,:]))

        return -self._des_step_len * x, z

    def _scale_helper(self, action, lower_lim, upper_lim):
        """Helper to linearly scale from [-1,1] to lower/upper limits. 
        This needs to be made general in case action range is not [-1,1]
        """
        new_a = lower_lim + 0.5 * (action + 1) * (upper_lim - lower_lim)
        # verify clip
        new_a = torch.clip(new_a, lower_lim, upper_lim)
        return new_a

    def get_CPG_RL_actions(self,actions,frequency_high,frequency_low,normal_forces):
        """ Map RL actions to CPG signals """
        MU_LOW = self.mu_low[0]
        MU_UPP = self.mu_up[0]
        MAX_STEP_LEN = self.max_step_len[0]
  
        device = self._device 
        a = torch.clip(actions, -1, 1)
        self._mu = self._scale_helper(a[:,:4],MU_LOW**2,MU_UPP**2) 
        self._omega_residuals = self._scale_helper(a[:,4:8],frequency_low,frequency_high)
        
        if "OFFSETX" in self._rl_task_string:
          offset = self._scale_helper( a[:,8:12],-0.07, 0.07) 
          self.update_offset_x(offset)

        self.integrate_oscillator_equations()
        
        x = torch.clip(self.X[:,0,:],MU_LOW,MU_UPP) 
        x = MAX_STEP_LEN * (x - MU_LOW) / (MU_UPP - MU_LOW)
 
        if "OFFSETX" in self._rl_task_string:
            x = -x * torch.cos(self.X[:,1,:]) - self._offset_x
            y = self.y
        else:
            x = -x * torch.cos(self.X[:,1,:]) 
            y = self.y  
        z = torch.where(torch.sin(self.X[:,1,:]) > 0, 
                        -self._robot_height + self._ground_clearance   * torch.sin(self.X[:,1,:]),
                        -self._robot_height + self._ground_penetration * torch.sin(self.X[:,1,:]))
        return x, y, z

    def integrate_oscillator_equations(self):
        device = self._device 
        X_dot = self.X_dot.clone() 
        d2X = self.d2X.clone()
        _a = 150
        dt = 0.001
        for _ in range(int(self._dt/dt)):
            d2X_prev = self.d2X.clone()
            X_dot_prev = self.X_dot.clone()
            X = self.X.clone()

            d2X = (_a * ( _a/4 * (torch.sqrt(self._mu) - X[:,0,:]) - X_dot_prev[:,0,:] )).unsqueeze(1)
            if self._couple:
                for i in range(4):
                    self._omega_residuals[:,i] += torch.sum(   X[:,0,:] * self._coupling_strength * torch.sin(X[:,1,:] - torch.remainder(self.X[:,1,i], (2*np.pi)) - self.PHI[i,:] ) ,2 )
            X_dot[:,1,:] = self._omega_residuals
            X_dot[:,0,:] = X_dot_prev[:,0,:] + (d2X_prev[:,0,:] + d2X[:,0,:]) * dt / 2
            self.X = X + (X_dot_prev + X_dot) * dt / 2 
            self.X_dot = X_dot
            self.d2X = d2X 
            self.X[:,1,:] = torch.remainder(self.X[:,1,:], (2*np.pi))

    def compute_inverse_kinematics(self,robot,legID, x, y, z):
        l1 = robot.hip_link_length_a1
        l2 = robot.thigh_link_length_a1
        l3 = robot.thigh_link_length_a1
    
        D = (y**2 + (-z)**2 - l1**2 +
        (-x)**2 - l2**2 - l3**2) / (
                 2 * l3 * l2)

        D = torch.clip(D, -1.0, 1.0)

        # check Right vs Left leg for hip angle
        sideSign = 1
        if legID == 0 or legID == 2:
          sideSign = -1

        knee_angle = torch.atan2(-torch.sqrt(1 - D**2), D)
        sqrt_component = y**2 + (-z)**2 - l1**2
        hip_roll_angle = -1*(-torch.atan2(z, y) - torch.atan2(
            torch.sqrt(sqrt_component), sideSign*l1*torch.ones_like(x)))
        hip_thigh_angle = torch.atan2(-x, torch.sqrt(sqrt_component)) -1* torch.atan2(
            l3 * torch.sin(knee_angle),
            l2 + l3 * torch.cos(knee_angle)) 
        output= torch.stack([hip_roll_angle, hip_thigh_angle, knee_angle], dim=-1)  
        return output
                    

