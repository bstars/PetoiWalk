from email.quoprimime import body_length
from os import walk
import time
from cvxpy import length
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sympy import true
from torch import le

class PetoiSimulate:
    def __init__(self) -> None:
        self.body_length = 10.5 # cm
        self.body_width = 10 # cm
        self.leg_length = 4.6 # cm
        self.foot_length = 4.6 # cm

        self.alphas = np.zeros([4])
        self.betas = np.zeros([4])

        # plotting initialization
        self.fig = plt.figure()
        
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_box_aspect([1,1,0.5])
        self.ax.set_aspect('auto')


        # shoulder coordinates
        self.shoulders = np.array([
            [ self.body_length / 2,  self.body_width / 2, 0],
            [-self.body_length / 2,  self.body_width / 2, 0],
            [-self.body_length / 2, -self.body_width / 2, 0],
            [ self.body_length / 2, -self.body_width / 2, 0]
        ])

    def leg_ik(self, xz:np.array):
        """
        inverse kinematics for one leg
        param: The xz coordinate of end-effector wrt the shoulder
        """


        l = np.sqrt(np.sum(xz**2))
        beta_tilde = np.arccos(
            (self.leg_length**2 + self.foot_length**2 - l**2) / 
            (2 * self.leg_length * self.foot_length)
        )
        beta = np.pi / 2 - beta_tilde

        gamma = np.arcsin( -xz[0]/l )
        alpha_tilde = np.arccos(
            (self.leg_length**2 + l**2 - self.foot_length**2) / 
            (2 * self.leg_length * l)
        )
        alpha = gamma + alpha_tilde

        return alpha, beta


    def render(self):

        plt.cla()
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None]
        )
        
        # plot body
        temp = np.stack([
            self.shoulders[0] / 2 + self.shoulders[3] / 2,
            self.shoulders[1] / 2 + self.shoulders[2] / 2
        ])
        self.ax.plot( temp[:,0], temp[:,1], temp[:,2], 'b-', linewidth=3 )

        temp = np.stack([
            self.shoulders[0],
            self.shoulders[3]
        ])
        self.ax.plot( temp[:,0], temp[:,1], temp[:,2], 'b-', linewidth=3 )

        temp = np.stack([
            self.shoulders[1],
            self.shoulders[2]
        ])
        self.ax.plot( temp[:,0], temp[:,1], temp[:,2], 'b-', linewidth=3 )

        # plot leg
        # this requires forward kinematics
        # note that joints and end-effectors are on the same x-z plane
        for i in range(4):


            leg = np.zeros([3, 3])
            leg[0] = self.shoulders[i]

        
            # knee_xz wrt shoulder
            knee_xz = np.array([
                -np.sin(self.alphas[i]), 
                -np.cos(self.alphas[i]), 
            ]) * self.leg_length

            # end_xz wrt shoulder
            end_xz = np.array([
                np.cos(self.betas[i]),
                np.sin(self.betas[i])
            ]) * self.foot_length
            end_xz = np.array([
                [np.cos(self.alphas[i]),  np.sin(self.alphas[i])],
                [-np.sin(self.alphas[i]),  np.cos(self.alphas[i])]
            ]) @ end_xz + knee_xz

            leg[1, 0] = knee_xz[0]
            leg[1, 2] = knee_xz[1]
            leg[2, 0] = end_xz[0]
            leg[2, 2] = end_xz[1]
            leg[1:] += leg[0]


            self.ax.scatter(leg[:,0], leg[:,1], leg[:,2])
            self.ax.plot(leg[:,0], leg[:,1], leg[:,2], linewidth=3)

        
        self.ax.set_xlim3d([-10, 8])
        self.ax.set_ylim3d([-5, 5])
        self.ax.set_zlim3d([-9, 0])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_aspect("auto")

        plt.pause(0.0001)
        # plt.show()


def test():
    robot = PetoiSimulate()
    # robot.alphas = np.ones([4]) * np.pi / 6

    alpha, beta = robot.leg_ik( np.array([4.6, -4.6]) )
    robot.alphas = np.ones([4]) * alpha
    robot.betas = np.ones([4]) * beta
    robot.render()

def walk_simulate(key_frames):
    robot = PetoiSimulate()

    angles = np.array([
        robot.leg_ik(f) for f in key_frames
    ])

    print(angles)
    
    i = 0
    while true:
        i = i % len(angles)
        robot.alphas[0] = angles[(i+len(angles)//2)%len(angles)][0]
        robot.alphas[2] = angles[(i+len(angles)//2)%len(angles)][0]
        robot.alphas[1] = angles[i][0]
        robot.alphas[3] = angles[i][0]

        robot.betas[0] = angles[(i+len(angles)//2)%len(angles)][1]
        robot.betas[2] = angles[(i+len(angles)//2)%len(angles)][1]
        robot.betas[1] = angles[i][1]
        robot.betas[3] = angles[i][1]

        robot.render()
        time.sleep(0.1)

        print(i, (i+len(angles)//2)%len(angles))

        i += 1
           

if __name__ == '__main__':
    


    key_frames = np.array([
        [1, -5.25],
        [0, -5.5],
        [-1, -5.75],
        [-2, -6],
        [-1, -5],
        [0,-5]
    ])

    walk_simulate(key_frames)
    # test()
    