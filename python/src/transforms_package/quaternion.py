from typing import Tuple
import numpy as np
import math 
from rotations import Rotations

# plot libraries
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

DEBUG=False

class Quaternion(Rotations):

    def __init__(self, data, deg=False, rvec=False, point=False, order='xyz', extrinsic=True):
        """Constructor takes a list which has 
        - quaternion values in the order x,y,z,w
        - rotation matrix 3x3 [https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/]
        - euler angles, by default considers angles in xyz order and an extrinsic rotation.

        :param data: [description]
        :type data: [type]
        :param deg: [description], defaults to False
        :type deg: bool, optional
        :param rvec: [description], defaults to False
        :type rvec: bool, optional
        :param point: [description], defaults to False
        :type point: bool, optional
        """        

        super().__init__()
        data = np.array(data)
        if len(data.shape) < 2:
            if rvec:
                angle = math.sqrt(data[0]**2 + data[1]**2 + data[2]**2)
                self.x = data[0]/angle * math.sin(angle/2)
                self.y = data[1]/angle * math.sin(angle/2)
                self.z = data[2]/angle * math.sin(angle/2)
                self.w = math.cos(angle/2)
                if DEBUG:
                    print("[INFO] Input considered an axis-angle")


            elif len(data) == 4:
                self.x = data[0]
                self.y = data[1]
                self.z = data[2]
                self.w = data[3]
                if DEBUG:
                    print("[INFO] Input considered Quaternion")

            elif len(data) == 3:
                if deg:
                    data = list(map(self.deg2rad, data))
                
                if order == "xyz":
                    data = np.matmul(np.matmul(self.Rz(data[2]),self.Ry(data[1])),self.Rx(data[0])) if extrinsic else np.matmul(np.matmul(self.Rx(data[0]), self.Ry(data[1])),self.Rz(data[2]))
                    self._quat_from_rotation(data)
                
                elif order == "zyx":
                    data = np.matmul(np.matmul(self.Rx(data[2]),self.Ry(data[1])),self.Rz(data[0])) if extrinsic else np.matmul(np.matmul(self.Rz(data[0]),self.Ry(data[1])),self.Rx(data[2]))
                    self._quat_from_rotation(data)
                
                else:
                    print("[ERROR]  Euler order not supported")

                if DEBUG:
                    print("[INFO] Input considered Euler angle")

        elif data.shape[0] == 3 and data.shape[1] == 3:
            self._quat_from_rotation(data)
            if DEBUG:
                    print("[INFO] Input considered Rotation Matrix")
        
        else: 
            print("[ERROR] invalid arguments")

        self.point = point
        self.magnitude = math.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)
        if not np.isclose(self.magnitude, 1.0, rtol=0) and not point:
            self.__dict__.update(self.normalize().__dict__)

    def __add__(self, q):
        return Quaternion(self.x+q.x,self.y+q.y, self.z+q.z, self.w+q.w)
    
    def __sub__(self, q):
        return Quaternion(self.x-q.x,self.y-q.y, self.z-q.z, self.w-q.w)
    
    def __str__(self):
        return "x:{}, y:{}, z:{}, w:{}\n".format(self.x, self.y, self.z, self.w)

    def __mul__(self, q):
        x =  self.x * q.w + self.y * q.z - self.z * q.y + self.w * q.x
        y = -self.x * q.z + self.y * q.w + self.z * q.x + self.w * q.y
        z =  self.x * q.y - self.y * q.x + self.z * q.w + self.w * q.z
        w = -self.x * q.x - self.y * q.y - self.z * q.z + self.w * q.w
        return Quaternion([x,y,z,w], point=q.point)
    
    def _quat_from_rotation(self,data):
        tr = np.trace(data)
        if (tr > 0):
            S = math.sqrt(tr+1.0) * 2
            self.w = 0.25 * S
            self.x = (data[2][1] - data[1][2]) / S
            self.y = (data[0][2] - data[2][0]) / S
            self.z = (data[1][0] - data[0][1]) / S 

        elif ((data[0][0] > data[1][1]) and (data[0][0] > data[2][2])):
            S = math.sqrt(1.0 + data[0][0] - data[1][1] - data[2][2]) * 2
            self.w = (data[2][1] - data[1][2]) / S
            self.x = 0.25 * S
            self.y = (data[0][1] + data[1][0]) / S 
            self.z = (data[0][2] + data[2][0]) / S

        elif (data[1][1] > data[2][2]):
            S = math.sqrt(1.0 + data[1][1] - data[0][0] - data[2][2]) * 2
            self.w = (data[0][2] - data[2][0]) / S
            self.x = (data[0][1] + data[1][0]) / S 
            self.y = 0.25 * S
            self.z = (data[1][2] + data[2][1]) / S
        else:
            S = math.sqrt(1.0 + data[2][2] - data[0][0] - data[1][1]) * 2
            self.w = (data[1][0] - data[0][1]) / S
            self.x = (data[0][2] + data[2][0]) / S
            self.y = (data[1][2] + data[2][1]) / S
            self.z = 0.25 * S

    def divide(self, scalar):
        """[summary]

        :param scalar: [description]
        :type scalar: [type]
        :return: [description]
        :rtype: [type]
        """

        assert not isinstance(scalar, Quaternion), "can only divide scalars"
        return Quaternion([self.x/scalar, self.y/scalar, self.z/scalar, self.w/scalar])
    
    def conjugate(self):
        """[summary]

        :return: [description]
        :rtype: [type]
        """        
        return Quaternion([-self.x, -self.y, -self.z, self.w])
    
    def inv(self):
        """[summary]

        :return: [description]
        :rtype: [type]
        """        
        return self.conjugate().normalize()
    
    def normalize(self):
        """[summary]
        """             
        return(self.divide(self.magnitude))
    
    def deg2rad(self, val):
        """[summary]

        :param val: [description]
        :type val: [type]
        :return: [description]
        :rtype: [type]
        """        
        return np.pi/180.0 * val
    
    def rad2deg(self, val):
        """[summary]

        :param val: [description]
        :type val: [type]
        :return: [description]
        :rtype: [type]
        """        
        return val *180.0/np.pi
    
    def to_euler(self, deg=False, order='zyx'):
        """Returns ZXY transform but as a list in  x,y,z order. In rad unless deg param is changed 

        :param deg: [description], defaults to False
        :type deg: bool, optional
        :return: [description]
        :rtype: [type]
        """        
        #ZYX - order only

        r = self.to_rotation_matrix()

        pitch = math.asin(- max(min(r[2][0],1), -1))
        yaw = math.atan2(r[1][0],r[0][0]) if math.fabs(r[2][0]) < 1 else math.atan2(-r[0][1],r[1][1])
        roll =  math.atan2(r[2][1],r[2][2]) if math.fabs(r[2][0]) < 1 else 0.0

        return [roll, pitch, yaw] if not deg else list(map(self.rad2deg,[roll, pitch, yaw]))

    
    def to_rotation_matrix(self):
        """[summary]

        :return: [description]
        :rtype: [type]
        """        
        sqw = self.w**2
        sqx = self.x**2
        sqy = self.y**2
        sqz = self.z**2

        # invs (inverse square length) is only required if quaternion is not already normalised
        invs = 1 / (sqx + sqy + sqz + sqw)
        m00 = ( sqx - sqy - sqz + sqw)*invs #since sqw + sqx + sqy + sqz =1/invs*invs
        m11 = (-sqx + sqy - sqz + sqw)*invs
        m22 = (-sqx - sqy + sqz + sqw)*invs

        tmp1 = self.x*self.y
        tmp2 = self.z*self.w
        m10 = 2.0 * (tmp1 + tmp2)*invs
        m01 = 2.0 * (tmp1 - tmp2)*invs

        tmp1 = self.x*self.z
        tmp2 = self.y*self.w
        m20 = 2.0 * (tmp1 - tmp2)*invs
        m02 = 2.0 * (tmp1 + tmp2)*invs
        tmp1 = self.y*self.z
        tmp2 = self.x*self.w
        m21 = 2.0 * (tmp1 + tmp2)*invs
        m12 = 2.0 * (tmp1 - tmp2)*invs

        return np.array([[m00,m01,m02],
                        [m10,m11,m12],
                        [m20,m21,m22]])
    
    def to_list(self):
        """[summary]

        :return: [description]
        :rtype: [type]
        """        
        return [self.x, self.y, self.z, self.w]
    
    def rotate(self,point):
        """[summary]

        :param point: [description]
        :type point: [type]
        :return: [description]
        :rtype: [type]
        """        
        q = Quaternion(point+[0], point=True)
        return (self * q) * self.conjugate()
    
    def to_rvec(self):
        angle = 2 * math.acos(self.w)
        x = self.x / math.sqrt(1-self.w*self.w)
        y = self.y / math.sqrt(1-self.w*self.w)
        z = self.z / math.sqrt(1-self.w*self.w)
        return [x*angle, y*angle, z*angle]





def plot_quaternions(ax, rotation=None, translation=[0.0,0.50,0], names=["q1"]):
    """[summary]

    :param ax: [description]
    :type ax: [type]
    :param rotation: [description], defaults to None
    :type rotation: [type], optional
    :param translation: [description], defaults to [0.0,0.50,0]
    :type translation: list, optional
    :param names: [description], defaults to ["test"]
    :type names: list, optional
    """    

    # origin- the axes you shall transform
    p1 =[1,0,0]
    p2 =[0,1,0]
    p3 =[0,0,1]
    p4 =[0,0,0]

    ax.plot([p4[0],p1[0]],[p4[1],p1[1]],[p4[2],p1[2]], alpha=0.25, color="r")
    ax.plot([p4[0],p2[0]],[p4[1],p2[1]],[p4[2],p2[2]], alpha=0.25, color="g")
    ax.plot([p4[0],p3[0]],[p4[1],p3[1]],[p4[2],p3[2]], alpha=0.25, color="b")
    ax.set_xlim(-2,2)
    ax.set_ylim(-2,2)
    ax.set_zlim(-2,2)

    if isinstance(rotation, Quaternion):
        q = rotation
        p4 = translation 

        p1 = np.array(q.rotate([1,0,0]).to_list()[:3]) + p4
        p2 = np.array(q.rotate([0,1,0]).to_list()[:3]) + p4
        p3 = np.array(q.rotate([0,0,1]).to_list()[:3]) + p4

        ax.plot([p4[0],p1[0]],[p4[1],p1[1]],[p4[2],p1[2]], color="r")
        ax.plot([p4[0],p2[0]],[p4[1],p2[1]],[p4[2],p2[2]], color="g")
        ax.plot([p4[0],p3[0]],[p4[1],p3[1]],[p4[2],p3[2]], color="b")
        ax.text(p4[0],p4[1],p4[2],names[0], fontsize=9,bbox=dict(facecolor="white", alpha=0.5, pad=0.1))

    elif isinstance(rotation, list):
        assert len(np.array(translation).shape) > 1, "Translation list not in right format"
        assert len(rotation) == len(translation), "Translation not provided for eah rotation"
        if  len(rotation) != len(names):
            names=["".join(["quat_",str(i)]) for i in range(len(rotation))]

        for i, q in enumerate(rotation):
            p4 = translation[i]

            p1 = np.array(q.rotate([1,0,0]).to_list()[:3]) + p4
            p2 = np.array(q.rotate([0,1,0]).to_list()[:3]) + p4
            p3 = np.array(q.rotate([0,0,1]).to_list()[:3]) + p4

            ax.plot([p4[0],p1[0]],[p4[1],p1[1]],[p4[2],p1[2]], color="r")
            ax.plot([p4[0],p2[0]],[p4[1],p2[1]],[p4[2],p2[2]], color="g")
            ax.plot([p4[0],p3[0]],[p4[1],p3[1]],[p4[2],p3[2]], color="b")
            ax.text(p4[0],p4[1],p4[2],names[i], fontsize=9,bbox=dict(facecolor="white", alpha=0.5, pad=0.1))

    ax.legend(["x","y","z"])

if __name__=="__main__":
    q1 = Quaternion( [-0.0880986,0.0015378,0.9959589,0.0173845] )
    q2 = Quaternion( [0.6420355,0.6648473,-0.2652181,0.2746413] )
    q3 = Quaternion( [-0.21263111527614184,-0.21263111527614184,0.6743797400305046,0.6743797063827515] )
    q4 = Quaternion( [0.21076748742763302,-0.21447855340451885,-0.6684690676326347, 0.6802390217781067] )

    t1 = [-1.6602861245985938, 0.0,1.732975221562235]
    t2 = [ -1.6532861245985937, 0.0, 1.6729752215622349]
    t3 = [ -0.11797776996071918, 1.0, 2.5067800929719004]
    t4 = [-0.13349286393377516, -1.0149782312015498, 2.5193396640847534]
    
    q5 = Quaternion([0,-90,0], deg=True)
    print(q5,q5.to_euler())

    fig, ax = plt.subplots(1)
    #plot_quaternions(ax, [q1,q2,q3,q4], [t1,t2,t3,t4], ["q1","q2","q3","q4"])
    plot_quaternions(ax, [q1,q2,q3,q4], [t1,t2,t3,t4])
    plt.show()
