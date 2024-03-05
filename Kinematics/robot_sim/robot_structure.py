import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from typing import Optional, List, Tuple


class Link:
    
    def __init__(self, rotation_axis: str, link_length: float, 
                 prelim_rotation: Optional[List] = None, inline=False) -> object:
        
        self.axis = rotation_axis
        self.length = link_length
        self.prelim_rotation = prelim_rotation
        self.inline = inline
            

    def _cx(self, x: float) -> np.ndarray: 
        return np.array([[1,0,0], 
                        [0,np.cos(x),-np.sin(x)], 
                        [0,np.sin(x),np.cos(x)]])

    def _cy(self, x:float) -> np.ndarray: 
        return np.array([[np.cos(x),0,np.sin(x)],
                        [0,1,0], 
                        [-np.sin(x),0,np.cos(x)]])

    def _cz(self, x: float) -> np.ndarray: 
        return np.array([[np.cos(x),-np.sin(x),0],
                        [np.sin(x),np.cos(x),0],
                        [0,0,1]])
    
    def build_h(self, theta: float):
    
        rotation_functions = [self._cx, self._cy, self._cz]
        rotation_letters = ["x", "y", "z"]

        # find rotation
        rotation = rotation_letters.index(self.axis)
        C = rotation_functions[rotation](theta)

        # add preliminary rotation
        if self.prelim_rotation:
            rotation = rotation_letters.index(self.prelim_rotation[0])
            C = C @ rotation_functions[rotation](self.prelim_rotation[1])

        # create translational component
        if self.inline:
            translation = np.array([self.length,0,0]).reshape(-1,1)
        else: 
            translation = np.array([self.length * np.cos(theta), 
                                    self.length*np.sin(theta), 
                                    0]).reshape(-1,1)
        # create H bottom row
        bottom_row = np.array([0,0,0,1])

        # combine parts
        self._H = np.hstack([C, translation])
        self._H = np.vstack([self._H, bottom_row])
    
        return self._H
    

class FourLinkArm:
    
    def __init__(self, link1: object, link2: object, link3: object, link4: object) -> object:

        self.kinematic_chain = [link1, link2, link3, link4]
    
    def get_coordinates(self, joint_angels: List[float]) -> List[np.ndarray]:
        
        joint_coordinates = []
        
        for i in range(len(self.kinematic_chain)):
            
            H_total = np.eye(4)
            
            for j in range(i+1):
                
                current_link = self.kinematic_chain[j]
                theta = joint_angels[j]
                H_link = current_link.build_h(theta)
                H_total =  H_total @ H_link

            joint_coordinates.append(H_total[:3, 3])
            
        joint_coordinates = np.array(joint_coordinates)
        joint_coordinates = np.vstack([np.array([0,0,0]), joint_coordinates])

        x_coords = joint_coordinates[:, 0]
        y_coords = joint_coordinates[:, 1]
        z_coords = joint_coordinates[:, 2]

        return x_coords, y_coords, z_coords
    
    
    def animate_arm(self, joint_angels: List[float] = [np.pi/2,0,np.pi/2,0]) -> None:
        
        x_coords, y_coords, z_coords = self.get_coordinates(joint_angels)
        
        fig = plt.figure(figsize=(10,8))

        ax1 = fig.add_subplot(4, 11, (1,33), projection = "3d")
        ax2 = fig.add_subplot(4, 11, (34,36))
        ax3 = fig.add_subplot(4, 11, (38,40))
        ax4 = fig.add_subplot(4, 11, (42,44))


        # plot default arm orientation
        ax1.scatter(0,0,0)
        plot = ax1.plot(x_coords, y_coords, z_coords, color="red")

        ax1.set_xlim([-55, 55])
        ax1.set_ylim([-55, 55])
        ax1.set_zlim([-55, 55])

        ax1.set_xlabel("x axis")
        ax1.set_ylabel("y axis")
        ax1.set_zlabel("z axis")

        theta1 = Slider(ax=ax2, label="1", valmin = 0, valmax = 2*np.pi, valinit = joint_angels[0])
        theta2 = Slider(ax=ax3, label="2", valmin = 0, valmax = 2*np.pi, valinit = joint_angels[1])
        theta3 = Slider(ax=ax4, label="3", valmin = 0, valmax = 2*np.pi, valinit = joint_angels[2])

        def update_theta1(theta):
            x_coords, y_coords, z_coords = self.get_coordinates([theta1.val, theta2.val, theta3.val, 0])
            ax1.cla()
            ax1.plot(x_coords, y_coords, z_coords, color="red")
            ax1.scatter(0,0,0)
            ax1.set_xlim([-55, 55])
            ax1.set_ylim([-55, 55])
            ax1.set_zlim([-55, 55])

            ax1.set_xlabel("x axis")
            ax1.set_ylabel("y axis")
            ax1.set_zlabel("z axis")

        theta1.on_changed(update_theta1)

        def update_theta2(theta):
            x_coords, y_coords, z_coords = self.get_coordinates([theta1.val, theta2.val, theta3.val, 0])
            ax1.cla()
            ax1.plot(x_coords, y_coords, z_coords, color="red")
            ax1.scatter(0,0,0)
            ax1.set_xlim([-55, 55])
            ax1.set_ylim([-55, 55])
            ax1.set_zlim([-55, 55])

            ax1.set_xlabel("x axis")
            ax1.set_ylabel("y axis")
            ax1.set_zlabel("z axis")

        theta2.on_changed(update_theta2)

        def update_theta3(theta):
            x_coords, y_coords, z_coords = self.get_coordinates([theta1.val, theta2.val, theta3.val, 0])
            ax1.cla()
            ax1.plot(x_coords, y_coords, z_coords, color="red")
            ax1.scatter(0,0,0)
            ax1.set_xlim([-55, 55])
            ax1.set_ylim([-55, 55])
            ax1.set_zlim([-55, 55])

            ax1.set_xlabel("x axis")
            ax1.set_ylabel("y axis")
            ax1.set_zlabel("z axis")

        theta3.on_changed(update_theta3)
        
        plt.show()
        