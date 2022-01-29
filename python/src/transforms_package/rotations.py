import numpy as np
import math 

# plot libraries
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



class Rotations:
    """Abstract class allows common functionality

    """
    def __init__(self):
        """Consructor
        """        
        pass

    def Rx(self,phi):
        """[summary]

        :param phi: [description]
        :type phi: [type]
        :return: [description]
        :rtype: [type]
        """        
        return np.array([[1, 0, 0],
                     [0, np.cos(phi), -np.sin(phi)],
                     [0, np.sin(phi), np.cos(phi)]])

    def Ry(self,theta):
        """[summary]

        :param theta: [description]
        :type theta: [type]
        :return: [description]
        :rtype: [type]
        """        
        return np.array([[np.cos(theta), 0, np.sin(theta)],
                         [0, 1, 0],
                         [-np.sin(theta), 0, np.cos(theta)]])

    def Rz(self, psi):
        """[summary]

        :param psi: [description]
        :type psi: [type]
        :return: [description]
        :rtype: [type]
        """        
        return np.array([[np.cos(psi), -np.sin(psi), 0],
                 [np.sin(psi), np.cos(psi), 0],
                 [0, 0, 1]])
    


if __name__== "__main__":

    r1 = Rotations(np.array([45.0, 45.0,0.0]), deg=True)
    r2 = Rotations(np.array([45.0, 45.0,0.0]), deg=True)
    fig, ax = plt.subplots(1)
    plot_rotation_mats(ax, [r1,r2],[[0,1,0],[0,-1,0]])
    plt.show()



        