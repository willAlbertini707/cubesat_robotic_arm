"""
William J. Albertini
4/8/24
Inverse kinematic solver for three link robotic arm.
How to use:

ThreeLinkArm uses various trig relationships to solve for 
joint angles and positions. An instance is create using three inputs:

a1, a2, a3: linkage lengths

After initialization, call the animate_arm() function to move
manipulator around its workspace. Three inputs are needed (given 
by the slider value):

x, y: end effector position in cartesian plane
si: end effector rotation angle (radians)

to do
--------
test cases for unreachable workspace
test cases for unattainable orientations


"""



import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from typing import Optional, List, Tuple
import math

class ThreeLinkArm:

    def __init__(self, a1: float, a2: float, a3: float):
        # assign links a length
        self.a1: int = a1
        self.a2: int = a2
        self.a3: int = a3

        # max length of fully entended arm
        self.max_reach = a1 + a2 + a3

    def second_joint_location(self, x3: float, y3: float, si: float) -> List[float]:
        """
        function second_joint_location:
        find location of wrist joint

        x3, y3: end effector coordinates
        si: end effector angle (radians)
        """
        x2 = x3 - self.a3 * np.cos(si)
        y2 = y3 - self.a3 * np.sin(si)
        return [x2, y2]
        
    def theta2(self, x2: float, y2: float) -> List[float]:
        """
        function theta2:
        angle of second joint relative to first joint. Finds elbow up/down 
        configuration, only elbow down is used.

        x2, y2: wrist joint location
        """
        # find elbow up config
        elbow_up = np.arccos((x2**2 + y2**2 - (self.a1**2 + self.a2**2)) / (2 * self.a1 * self.a2))

        # elbow down is opposite of elbow up
        elbow_down = -elbow_up

        return [elbow_up, elbow_down]
    
    def theta1(self, x2: float, y2: float, theta2: float) -> float:
        """
        function theta1:
        find angle of first link (radians)

        x2, y2: wrist joint location
        theta2: relative angle of second link (radians)
        """
        gamma = np.arctan(y2 / x2)
        beta = np.arctan(self.a2 * np.sin(theta2) / (self.a1 + self.a2*np.cos(theta2)))

        if x2 < 0 and y2 >= 0:
            gamma += np.pi
        elif x2 < 0 and y2 < 0:
            gamma += np.pi
        elif x2 > 0 and y2 < 0:
            gamma += 2*np.pi

        theta1 = gamma - beta
        
        if theta1 < 0:
            theta1 += 2*np.pi

        return theta1
    
    def first_joint_location(self, theta1: float) -> List[float]:
        """
        function first_joint_location:
        find location of elbow joint

        theta1: angle of first link (radians)
        """
        # use simple trig and theta1 to find location of joint
        x1 = self.a1 * np.cos(theta1)
        y1 = self.a1 * np.sin(theta1)
        return [x1, y1]

    def joint_coordinates(self, x3: float, y3: float, si: float) -> Tuple[List[float]]:
        """
        function joint_coordinates:
        solve for all joint coordinates (uses elbow down configuration)

        x3, y3: end effector position
        si: end effector orientation
        """

        # find location of second joint
        x2, y2 = self.second_joint_location(x3, y3, si)
        # use second joint to find elbow down solution (angles)

        theta2 = self.theta2(x2, y2)[0]
        # if solution is possible, assign private variables
        if not math.isnan(theta2):
            self._theta2 = theta2
            self._x2 = x2
            self._y2 = y2

            self._si = si
            self._x3 = x3
            self._y3 = y3
    
        self._theta1 = self.theta1(self._x2, self._y2, self._theta2)

        # find first joint location
        self._x1, self._y1 = self.first_joint_location(self._theta1)
        
        return ([0, self._x1, self._x2, self._x3], [0, self._y1, self._y2, self._y3])

    def animate_arm(self) -> None:

        # start in default position (full length across x-axis)
        x_coords, y_coords = self.joint_coordinates(self.max_reach, 0, 0)

        # create figure
        fig = plt.figure(figsize=(10,8))

        # create main plot
        ax1 = fig.add_subplot(4, 11, (1,33))

        # create slider axes
        ax2 = fig.add_subplot(4, 11, (34,36))
        ax3 = fig.add_subplot(4, 11, (38,40))
        ax4 = fig.add_subplot(4, 11, (42,44))

        # plot default arm orientation
        plot = ax1.plot(x_coords, y_coords, color="blue")
        ax1.set_xlim([-self.max_reach, self.max_reach])
        ax1.set_ylim([-self.max_reach, self.max_reach])

        ax1.set_xlabel("x")
        ax1.set_ylabel("y")

        x3 = Slider(ax=ax2, label="x", valmin = -self.max_reach, valmax = self.max_reach, valinit = self.max_reach)
        y3 = Slider(ax=ax3, label="y", valmin = -self.max_reach, valmax = self.max_reach, valinit = 0)
        si = Slider(ax=ax4, label="si", valmin = 0, valmax = 2*np.pi, valinit = 0)

        def update_x3(pos):
            x_coords, y_coords = self.joint_coordinates(x3.val, y3.val, si.val)
            ax1.cla()
            ax1.plot(x_coords, y_coords, color="blue")
            ax1.set_xlim([-self.max_reach, self.max_reach])
            ax1.set_ylim([-self.max_reach, self.max_reach])

            ax1.set_xlabel("x axis")
            ax1.set_ylabel("y axis")
            self.print_arm_values()

        x3.on_changed(update_x3)

        def update_y3(pos):
            x_coords, y_coords = self.joint_coordinates(x3.val, y3.val, si.val)
            ax1.cla()
            ax1.plot(x_coords, y_coords, color="blue")
            ax1.set_xlim([-self.max_reach, self.max_reach])
            ax1.set_ylim([-self.max_reach, self.max_reach])

            ax1.set_xlabel("x axis")
            ax1.set_ylabel("y axis")
            self.print_arm_values()

        y3.on_changed(update_y3)

        def update_si(theta):
            x_coords, y_coords = self.joint_coordinates(x3.val, y3.val, si.val)
            ax1.cla()
            ax1.plot(x_coords, y_coords, color="blue")
            ax1.set_xlim([-self.max_reach, self.max_reach])
            ax1.set_ylim([-self.max_reach, self.max_reach])

            ax1.set_xlabel("x axis")
            ax1.set_ylabel("y axis")

            self.print_arm_values()

        si.on_changed(update_si)
        
        plt.show()

    def print_arm_values(self) -> None:
        
        try:
            
            print(f"""
joint angles: {self._theta1}, {self._theta2}, {self._si}
""")    
        except:
            print("Orientation is not possible")