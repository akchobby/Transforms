import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math 
import cv2



class Quaternion:

    def __init__(self, data, deg=False, rvec=False):
        """
        Function takes a list which has 
        - quaternion values in the order x,y,z,w
        - rotation matrix 3x3 [https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/]
        - euler angles in order of ZYX rotation

        """
        data = np.array(data)
        if len(data.shape) < 2:
            if rvec:
                angle = math.sqrt(data[0]**2 + data[1]**2 + data[2]**2)
                self.x = data[0]/angle * math.sin(angle/2)
                self.y = data[1]/angle * math.sin(angle/2)
                self.z = data[2]/angle * math.sin(angle/2)
                self.w = math.cos(angle/2)


            elif len(data) == 4:
                self.x = data[0]
                self.y = data[1]
                self.z = data[2]
                self.w = data[3]

            elif len(data) == 3:
                if deg:
                    data = map(self.deg2rad, data)
                cy = math.cos(data[0] * 0.5)
                sy = math.sin(data[0] * 0.5)
                cp = math.cos(data[1] * 0.5)
                sp = math.sin(data[1] * 0.5)
                cr = math.cos(data[2] * 0.5)
                sr = math.sin(data[2] * 0.5)
                self.w = cr * cp * cy + sr * sp * sy
                self.x = sr * cp * cy - cr * sp * sy
                self.y = cr * sp * cy + sr * cp * sy
                self.z = cr * cp * sy - sr * sp * cy

        elif data.shape[0] == 3 and data.shape[1] == 3:
            tr = np.trace(data)
            if (tr > 0):
                S = math.sqrt(tr+1.0) * 2; 
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
        
        else: 
            print("[ERROR] invalid arguments")

        self.magnitude = math.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)
    
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
        return Quaternion([x,y,z,w])

    def divide(self, scalar):
        assert not isinstance(scalar, Quaternion), "can only divide scalars"
        return Quaternion([self.x/scalar, self.y/scalar, self.z/scalar, self.w/scalar])
    
    def conjugate(self):
        return Quaternion([-self.x, -self.y, -self.z, self.w])
    
    def inv(self):
        return self.conjugate().normalize()
    
    def normalize(self):
        return(self.divide(self.magnitude))
    
    def deg2rad(self, val):
        return np.pi/180.0 * val
    
    def rad2deg(self, val):
        return val *180.0/np.pi
    
    def to_euler(self, deg=False):
        """
        Returns ZXY transform but as a list in  x,y,z order. In rad unless deg param is changed 
        """
        # roll (x-axis rotation)
        sinr_cosp = 2 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1 - 2 * (self.x * self.x + self.y * self.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (self.w * self.y - self.z * self.x)
        if (math.fabs(sinp) >= 1):
            pitch = math.copysign(np.pi / 2, sinp); # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1 - 2 * (self.y * self.y + self.z * self.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw] if not deg else map(self.rad2deg,[roll, pitch, yaw])
    
    def to_rotation_matrix(self):
        pass
    
    def to_list(self):
        return [self.x, self.y, self.z, self.w]
    
    def rotate(self,point):
        q = Quaternion(point+[0])
        return (self * q) * self.conjugate()
    
    def to_rvec(self):
        angle = 2 * math.acos(self.w)
        x = self.x / math.sqrt(1-self.w*self.w)
        y = self.y / math.sqrt(1-self.w*self.w)
        z = self.z / math.sqrt(1-self.w*self.w)
        return [x*angle, y*angle, z*angle]





def plot_3d(ax, rotation=None, translation=[0.0,0.50,0]):

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


    # TO DO : set these correctly
    if isinstance(rotation, Quaternion):
        q = rotation
        p4 = np.array(q.rotate(translation).to_list()[:3])

        p1 = np.array(q.rotate(p1).to_list()[:3]) + p4
        p2 = np.array(q.rotate(p2).to_list()[:3]) + p4
        p3 = np.array(q.rotate([p3]).to_list()[:3]) + p4

        ax.plot([p4[0],p1[0]],[p4[1],p1[1]],[p4[2],p1[2]], color="r")
        ax.plot([p4[0],p2[0]],[p4[1],p2[1]],[p4[2],p2[2]], color="g")
        ax.plot([p4[0],p3[0]],[p4[1],p3[1]],[p4[2],p3[2]], color="b")

    elif isinstance(rotation, list):
        assert len(np.array(translation).shape) > 1, "Translation list not in right format"
        assert len(rotation) == len(translation), "Translation not provided for eah rotation"

        for i, q in enumerate(rotation):
            p4 = np.array(q.rotate(translation[i]).to_list()[:3])

            p1 = np.array(q.rotate([1,0,0]).to_list()[:3]) + p4
            p2 = np.array(q.rotate([0,1,0]).to_list()[:3]) + p4
            p3 = np.array(q.rotate([0,0,1]).to_list()[:3]) + p4

            ax.plot([p4[0],p1[0]],[p4[1],p1[1]],[p4[2],p1[2]], color="r")
            ax.plot([p4[0],p2[0]],[p4[1],p2[1]],[p4[2],p2[2]], color="g")
            ax.plot([p4[0],p3[0]],[p4[1],p3[1]],[p4[2],p3[2]], color="b")

    ax.legend(["x","y","z"])






    
if __name__=="__main__" :
    
    rotMat = np.array([[0,-1,0],[0,0,-1],[1,0,0]], dtype=np.float64)
    rotMat_cam0 = np.array([[-0.9995930750515504, 0.028180516203601146, -0.004420725673603042 ],
                            [0.00408373330785272, -0.012004702762037862, -0.9999196018850028 ],
                            [-0.028231320040922887, -0.9995307627172317, 0.01188473603633216 ]], dtype=np.float64)


    t_cam0 = [0.06508125947885363, -0.12425698102638041, -0.08094531518938053]



    rotMat_cam1 = np.array([[-0.999991980508952, 0.0032425627112567875, -0.002350469069725772 ],
                            [0.0023816581389216724, 0.009648555620498319, -0.9999506153200506 ],
                            [-0.0032197239467815653, -0.9999481942388326, -0.009656200919865136]], dtype=np.float64)

    t_cam1= [-0.05880865244539242, -0.1215735034443526, -0.0856024019820022]

    q1 = Quaternion(rotMat_cam0)
    q2 = Quaternion(rotMat_cam1)

    print(q1.to_rvec())
    print(q2.to_rvec())


    fig, ax = plt.subplots(1)
    plot_3d(ax, [q1,q2], [t_cam0, t_cam1])
    plt.show(fig)
    



