import numpy as np
import math 
from rotations import Rotations

# plot libraries
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



class RotationMatrix(Rotations):
    def __init__(self, input, deg=False, order="xyz", extrinsic=True):
        """Consructor
        source for equations

        https://www.cs.utexas.edu/~theshark/courses/cs354/lectures/cs354-14.pdf

        :param input: [description]
        :type input: [type]
        :param deg: [description], defaults to False
        :type deg: bool, optional
        :param order: [description], defaults to "xyz"
        :type order: str, optional
        :param extrinsic: [description], defaults to True
        :type extrinsic: bool, optional
        """   

        super().__init__()     

        assert isinstance(input,np.ndarray) , "Input not numpy array, input numpy array"
        if len(input.shape) == 2:
            self.rotation_matrix = input
            print("Rotation matrix is input")
        
        else:
            if deg:
                input = list(map(np.deg2rad, input))

            if order == "xyz":
                self.rotation_matrix = np.matmul(np.matmul(self.Rz(input[2]),self.Ry(input[1])),self.Rx(input[0])) if extrinsic else np.matmul(np.matmul(self.Rx(input[0]), self.Ry(input[1])),self.Rz(input[2]))
            elif order == "zyx":
                self.rotation_matrix = np.matmul(np.matmul(self.Rx(input[2]),self.Ry(input[1])),self.Rz(input[0])) if extrinsic else np.matmul(np.matmul(self.Rz(input[0]),self.Ry(input[1])),self.Rx(input[2]))
            else:
                print("Wrong order, provide in using 'xyz'")
    
    def rotate(self,point):
        """[summary]

        :param point: [description]
        :type point: [type]
        :return: [description]
        :rtype: [type]
        """        
        assert isinstance(point, np.ndarray) , "Point not numpy array, input numpy array"
        return np.matmul(self.rotation_matrix, point.T) if (point.shape[1] == 3)  else np.matmul(self.rotation_matrix,point)


def plot_rotation_mats(ax, rotation_mat=None, translation=[0,0.05,0], names=["test"]):
    """[summary]

    :param ax: [description]
    :type ax: [type]
    :param rotation_mat: [description], defaults to None
    :type rotation_mat: [type], optional
    :param translation: [description], defaults to [0,0.05,0]
    :type translation: list, optional
    """    
    # origin- the axes you shall transform
    p1 =[1,0,0]
    p2 =[0,1,0]
    p3 =[0,0,1]
    p4 =[0,0,0]

    ax = plt.axes(projection='3d', azim=170, elev=20)
    ax.plot([p4[0],p1[0]],[p4[1],p1[1]],[p4[2],p1[2]], alpha=0.25, color="r")
    ax.plot([p4[0],p2[0]],[p4[1],p2[1]],[p4[2],p2[2]], alpha=0.25, color="g")
    ax.plot([p4[0],p3[0]],[p4[1],p3[1]],[p4[2],p3[2]], alpha=0.25, color="b")
    ax.set_xlim(-2,2)
    ax.set_ylim(-2,2)
    ax.set_zlim(-2,2)

    if isinstance(rotation_mat, RotationMatrix):
        p4 = translation

        p1 =  (rotation_mat.rotate(np.array([[1,0,0]]).T) + np.array([translation]).T).T[0]
        p2 =  (rotation_mat.rotate(np.array([[0,1,0]]).T) + np.array([translation]).T).T[0]
        p3 =  (rotation_mat.rotate(np.array([[0,0,1]]).T) + np.array([translation]).T).T[0]

        ax.plot([p4[0],p1[0]],[p4[1],p1[1]],[p4[2],p1[2]], color="r")
        ax.plot([p4[0],p2[0]],[p4[1],p2[1]],[p4[2],p2[2]], color="g")
        ax.plot([p4[0],p3[0]],[p4[1],p3[1]],[p4[2],p3[2]], color="b")

    elif isinstance(rotation_mat, list):

        assert len(rotation_mat) == len(translation), 'Mismatch of translation vector for rotation'
        if  len(rotation_mat) != len(names):
            names=["".join(["rot_",str(i)]) for i in range(len(rotation_mat))]

        for i,rotation in enumerate(rotation_mat):
            p4 = translation[i]

            p1 =  (rotation.rotate(np.array([[1,0,0]]).T) + np.array([p4]).T).T[0]
            p2 =  (rotation.rotate(np.array([[0,1,0]]).T) + np.array([p4]).T).T[0]
            p3 =  (rotation.rotate(np.array([[0,0,1]]).T) + np.array([p4]).T).T[0]

            ax.plot([p4[0],p1[0]],[p4[1],p1[1]],[p4[2],p1[2]], color="r")
            ax.plot([p4[0],p2[0]],[p4[1],p2[1]],[p4[2],p2[2]], color="g")
            ax.plot([p4[0],p3[0]],[p4[1],p3[1]],[p4[2],p3[2]], color="b")
            ax.text(p4[0],p4[1],p4[2],names[i], fontsize=9,bbox=dict(facecolor="white", alpha=0.5, pad=0.1))

    else:
        p4 = translation

        p1 = (np.matmul(np.array(rotation_mat), np.array([[1,0,0]]).T) + np.array([translation]).T).T[0]
        p2 =  (np.matmul(np.array(rotation_mat), np.array([[0,1,0]]).T) + np.array([translation]).T).T[0]
        p3 =  (np.matmul(np.array(rotation_mat), np.array([[0,0,1]]).T) + np.array([translation]).T).T[0]

        ax.plot([p4[0],p1[0]],[p4[1],p1[1]],[p4[2],p1[2]], color="r")
        ax.plot([p4[0],p2[0]],[p4[1],p2[1]],[p4[2],p2[2]], color="g")
        ax.plot([p4[0],p3[0]],[p4[1],p3[1]],[p4[2],p3[2]], color="b")
    ax.legend(["x","y","z"])


if __name__== "__main__":

    r1 = RotationMatrix(np.array([45.0, 45.0,0.0]), deg=True)
    r2 = RotationMatrix(np.array([45.0, 45.0,0.0]), deg=True)
    fig, ax = plt.subplots(1)
    plot_rotation_mats(ax, [r1,r2],[[0,1,0],[0,-1,0]])
    plt.show()



        