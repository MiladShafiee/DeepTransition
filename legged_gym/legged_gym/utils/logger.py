# License: see [LICENSE, LICENSES/legged_gym/LICENSE]

import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
from multiprocessing import Process, Value

class Logger:
    def __init__(self, dt):
        self.state_log = defaultdict(list)
        self.rew_log = defaultdict(list)
        self.dt = dt
        self.num_episodes = 0
        self.plot_process = None

    def log_state(self, key, value):
        self.state_log[key].append(value)

    def log_states(self, dict):
        for key, value in dict.items():
            self.log_state(key, value)

    def log_rewards(self, dict, num_episodes):
        for key, value in dict.items():
            if 'rew' in key:
                self.rew_log[key].append(value.item() * num_episodes)
        self.num_episodes += num_episodes

    def reset(self):
        self.state_log.clear()
        self.rew_log.clear()

    def plot_states(self):
        self.plot_process = Process(target=self._plot)
        self.plot_process.start()

    def _plot(self):
        nb_rows = 3
        nb_cols = 3
        fig, axs = plt.subplots(nb_rows, nb_cols)
        for key, value in self.state_log.items():
            time = np.linspace(0, len(value)*self.dt, len(value))
            break
        log= self.state_log
        # plot joint targets and measured positions
        a = axs[1, 0]
        if log["dof_pos"]: a.plot(time, log["dof_pos"], label='measured')
        #if log["dof_pos_target"]: a.plot(time, log["dof_pos_target"], label='target')
        a.set(xlabel='time [s]', ylabel='Position [rad]', title='DOF Position')
        a.legend()
        # plot joint velocity
        a = axs[1, 1]
        if log["dof_vel"]: a.plot(time, log["dof_vel"], label='measured')
        if log["dof_vel_target"]: a.plot(time, log["dof_vel_target"], label='target')
        a.set(xlabel='time [s]', ylabel='Velocity [rad/s]', title='Joint Velocity')
        a.legend()
        # plot base vel x
        a = axs[0, 0]
        if log["base_vel_x"]: a.plot(time, log["base_vel_x"], label=' base velx measured')
        if log["base_x"]: a.plot(time, log["base_x"], label=' base x measured')
        # if log["command_x"]: a.plot(time, log["command_x"], label='commanded')
        # a.set(xlabel='time [s]', ylabel='base lin vel [m/s]', title='Base velocity x')
        # a.legend()
        # # plot base vel y
        # a = axs[0, 1]
        if log["base_vel_y"]: a.plot(time, log["base_vel_y"], label='measured')
        if log["command_y"]: a.plot(time, log["command_y"], label='commanded')
        a.set(xlabel='time [s]', ylabel='base lin vel [m/s]', title='Base velocity')
        a.legend()
        # plot base vel yaw
        a = axs[0, 1]
        if log["base_vel_yaw"]: a.plot(time, log["base_vel_yaw"], label='measured')
        if log["command_yaw"]: a.plot(time, log["command_yaw"], label='commanded')
        a.set(xlabel='time [s]', ylabel='base ang vel [rad/s]', title='Base velocity yaw')
        a.legend()
        # plot base vel z
        # a = axs[1, 2]
        a = axs[0, 2]
        if log["base_vel_z"]: a.plot(time, log["base_vel_z"], label='z vel')
        if log["base_z"]: a.plot(time, log["base_z"], label='z pos')
        if log["base_vel_x_inertial"]: a.plot(time, log["base_vel_x_inertial"], label='x vel inertia')

        a.set(xlabel='time [s]', ylabel='base lin vel [m/s]', title='Base z')
        a.legend()
        # plot contact forces
        a = axs[2, 0]
        if log["contact_forces_z"]:
            forces = np.array(log["contact_forces_z"])
            for i in range(forces.shape[1]):
                a.plot(time, forces[:, i], label=f'force {i}')
        a.set(xlabel='time [s]', ylabel='Forces z [N]', title='Vertical Contact forces')
        a.legend()
        # plot torque/vel curves
        # a = axs[2, 1]
        # if log["dof_vel"]!=[] and log["dof_torque"]!=[]: a.plot(log["dof_vel"], log["dof_torque"], 'x', label='measured')
        # a.set(xlabel='Joint vel [rad/s]', ylabel='Joint Torque [Nm]', title='Torque/velocity curves')
        # a.legend()
        # plot torques
        # a = axs[2, 2]
        # if log["dof_torque"]!=[]: a.plot(time, log["dof_torque"], label='measured')
        # a.set(xlabel='time [s]', ylabel='Joint Torque [Nm]', title='Torque')
        # a.legend()
        # plt.show()

        # plot CPG r, theta
        a = axs[2, 1]
        # if log["cpg_r0"]!=[]: 
        #     a.plot(time, log["cpg_r0"], label='r0')
        #     # a.plot(time, log["cpg_th0"], label='th0')
        #     a.plot(time, log["cpg_r1"], label='r1')
        #     # a.plot(time, log["cpg_th1"], label='th1')
        #     a.plot(time, log["cpg_r2"], label='r2')
        #     # a.plot(time, log["cpg_th2"], label='th2')
        #     a.plot(time, log["cpg_r3"], label='r3')
        #     # a.plot(time, log["cpg_th3"], label='th3')
        #     a.plot(time, log["cpg_r4"], label='r4')
        #     a.plot(time, log["cpg_r5"], label='r5')
        #     a.plot(time, log["cpg_r6"], label='r6')
        #     a.plot(time, log["cpg_r7"], label='r7')
        # a.set(xlabel='time [s]',title='CPG x 0') 
        # a.legend()

        if log["veloVec"]!=[]:
            a.plot(time,log["veloVec"])
            a.set(xlabel='time [s]',title='All Amplitudes')
            a.legend() 

        a = axs[2, 2]
        # if log["cpg_dr0"]!=[]: 
        #     a.plot(time, log["cpg_dr0"], label='dr0')
        #     a.plot(time, log["cpg_dth0"], label='dth0')
        # a.set(xlabel='time [s]',title='CPG x_dot 0')
        # a.legend()
        # if log["cpg_th0"]!=[]: 
        #     a.plot(time, log["cpg_th0"], label='th0')
        #     a.plot(time, log["cpg_th1"], label='th1')
        #     a.plot(time, log["cpg_th2"], label='th2')
        #     a.plot(time, log["cpg_th3"], label='th3')
        #     a.plot(time, log["cpg_th4"], label='th4')
        #     a.plot(time, log["cpg_th5"], label='th5')
        #     a.plot(time, log["cpg_th6"], label='th6')
        #     a.plot(time, log["cpg_th7"], label='th7')
        # a.set(xlabel='time [s]',title='CPG x 0') 
        # a.legend()

        if log["cpg_theta"]!=[]:
            a.plot(time,log["cpg_theta"])
            a.set(xlabel='time [s]',title='All Phases')
            a.legend() 

        a = axs[1, 2]
        # if log["cpg_psi0"]!=[]: 
        #     a.plot(time, log["cpg_psi0"], label='psi0')
        #     a.plot(time, log["cpg_dpsi0"], label='dpsi0')
        # a.set(xlabel='time [s]',title='CPG PSI 0') 
        # a.legend()

        a = axs[1, 2]
        if log["cpg_xoff0"]!=[]: 
            a.plot(time, log["cpg_xoff0"], label='xoff0')
            a.plot(time, log["cpg_xoff1"], label='xoff1')
            a.plot(time, log["cpg_xoff2"], label='xoff2')
            a.plot(time, log["cpg_xoff3"], label='xoff3')
            a.plot(time, log["cpg_xoff4"], label='xoff4')
            a.plot(time, log["cpg_xoff5"], label='xoff5')
            a.plot(time, log["cpg_xoff6"], label='xoff6')
            a.plot(time, log["cpg_xoff7"], label='xoff7')
            a.set(xlabel='time [s]',title='CPG Xoff') 
            a.legend()
        elif log["actions"]!=[]:
            a.plot(time,log["actions"])
            a.set(xlabel='time [s]',title='Actions') 

        plt.show()




        fig6 = plt.figure()
        plt.subplot(2, 2, 1)

        mil=plt.plot(time,log["veloVec1"])
        plt.legend(iter(mil),('Little-Dog', 'Spot-Micro', 'Solo' ,'Mini-Cheetah'))
        ax = plt.gca()
        ax.set_ylim(0,1.72)
        ax.set_xlabel('time (s)')
        ax.set_ylabel('Base Velocity (m/s)')

        plt.subplot(2, 2, 2)
        mil=plt.plot(time,log["veloVec2"])
        plt.legend(iter(mil),( 'A1', 'Go1', 'Aliengo' ,'Laikago'))
        ax = plt.gca()
        ax.set_ylim(0,1.72)
        ax.set_xlabel('time (s)')
        ax.set_ylabel('Base Velocity (m/s)')

        plt.subplot(2, 2, 3)
        mil=plt.plot(time,log["veloVec3"])
        plt.legend(iter(mil),('Anymal-B', 'Anymal-C', 'Spot' ,'B1'))
        ax = plt.gca()
        ax.set_ylim(0,1.72)
        ax.set_xlabel('time (s)')
        ax.set_ylabel('Base Velocity (m/s)')

        plt.subplot(2, 2, 4)
        mil=plt.plot(time,log["veloVec4"])
        plt.legend(iter(mil),( 'HYQ', 'Dog 1', 'Dog 2' ,'Dog 3'))
        ax = plt.gca()
        ax.set_ylim(0,1.72)
        ax.set_xlabel('time (s)')
        ax.set_ylabel('Base Velocity (m/s)')
        plt.show()

        fig7 = plt.figure()
        plt.subplot(2, 2, 1)
        mil=plt.plot(time,log["omega1"])
        plt.legend(iter(mil),('Little-Dog', 'Spot-Micro', 'Solo' ,'Mini-Cheetah'))
        ax = plt.gca()
        ax.set_xlabel('time (s)')
        ax.set_ylabel('CPG Frequency (rad/s)')

        plt.subplot(2, 2, 2)
        mil=plt.plot(time,log["omega2"])
        plt.legend(iter(mil),( 'A1', 'Go1', 'Aliengo' ,'Laikago'))
        ax = plt.gca()
        ax.set_xlabel('time (s)')
        ax.set_ylabel('CPG Frequency (rad/s)')


        plt.subplot(2, 2, 3)
        mil=plt.plot(time,log["omega3"])
        plt.legend(iter(mil),('Anymal-B', 'Anymal-C', 'Spot' ,'B1'))
        ax = plt.gca()
        ax.set_xlabel('time (s)')
        ax.set_ylabel('CPG Frequency (rad/s)')
 

        plt.subplot(2, 2, 4)
        mil=plt.plot(time,log["omega4"])
        plt.legend(iter(mil),( 'HYQ', 'Dog 1', 'Dog 2' ,'Dog 3'))
        ax = plt.gca()
        ax.set_xlabel('time (s)')
        ax.set_ylabel('CPG Frequency (rad/s)')
        plt.legend()
        plt.show()



        fig7 = plt.figure()
        plt.subplot(2, 2, 1)
        mil=plt.plot(time,log["amplit1"])
        plt.legend(iter(mil),('Little-Dog', 'Spot-Micro', 'Solo' ,'Mini-Cheetah'))
        ax = plt.gca()
        ax.set_xlabel('time (s)')
        ax.set_ylabel('CPG Amplitude')

        plt.subplot(2, 2, 2)
        mil=plt.plot(time,log["amplit2"])
        plt.legend(iter(mil),( 'A1', 'Go1', 'Aliengo' ,'Laikago'))
        ax = plt.gca()
        ax.set_xlabel('time (s)')
        ax.set_ylabel('CPG Amplitude')

        plt.subplot(2, 2, 3)
        mil=plt.plot(time,log["amplit3"])
        plt.legend(iter(mil),('Anymal-B', 'Anymal-C', 'Spot' ,'B1'))
        ax = plt.gca()
        ax.set_xlabel('time (s)')
        ax.set_ylabel('CPG Amplitude')

        plt.subplot(2, 2, 4)
        mil=plt.plot(time,log["amplit4"])
        plt.legend(iter(mil),( 'HYQ', 'Dog 1', 'Dog 2' ,'Dog 3'))
        ax = plt.gca()
        ax.set_xlabel('time (s)')
        ax.set_ylabel('CPG Amplitude')
        plt.show()




    def print_rewards(self):
        print("Average rewards per second:")
        for key, values in self.rew_log.items():
            mean = np.sum(np.array(values)) / self.num_episodes
            print(f" - {key}: {mean}")
        print(f"Total number of episodes: {self.num_episodes}")
    
    def __del__(self):
        if self.plot_process is not None:
            self.plot_process.kill()