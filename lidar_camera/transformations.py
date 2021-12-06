import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math 
#import cv2



class Quaternion:

    def __init__(self, data, deg=False, rvec=False, point=False):
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
                    data = list(map(self.deg2rad, data))
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

        #ZYX - surrent case if xyz needed then : z = x, x=z
        # roll (x-axis rotation)
        sinr_cosp = 2 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1 - 2 * (self.x * self.x + self.y * self.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (self.w * self.y - self.z * self.x)
        if (math.fabs(sinp) >= 1):
            pitch = math.copysign(np.pi / 2, sinp) # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1 - 2 * (self.y * self.y + self.z * self.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw] if not deg else list(map(self.rad2deg,[roll, pitch, yaw]))

    
    def to_rotation_matrix(self):
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
        return [self.x, self.y, self.z, self.w]
    
    def rotate(self,point):
        q = Quaternion(point+[0], point=True)
        return (self * q) * self.conjugate()
    
    def to_rvec(self):
        angle = 2 * math.acos(self.w)
        x = self.x / math.sqrt(1-self.w*self.w)
        y = self.y / math.sqrt(1-self.w*self.w)
        z = self.z / math.sqrt(1-self.w*self.w)
        return [x*angle, y*angle, z*angle]





def plot_3d(ax, rotation=None, translation=[0.0,0.50,0], names=["test"]):

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




class Rotations:
    def __init__(self, angles, deg=True, order="xyz", extrinsic=True):
        """
        TODO : documentation of source equations
 
        """
        if deg:
            angles = list(map(np.deg2rad, angles))
        if order == "xyz":
            self.rotation_matrix = np.matmul(np.matmul(self.Rz(angles[2]),self.Ry(angles[1])),self.Rx(angles[0])) if extrinsic else np.matmul(np.matmul(self.Rx(angles[0]), self.Ry(angles[1])),self.Rz(angles[2]))
        elif order == "zyx":
            self.rotation_matrix = np.matmul(np.matmul(self.Rx(angles[2]),self.Ry(angles[1])),self.Rz(angles[0])) if extrinsic else np.matmul(np.matmul(self.Rz(angles[0]),self.Ry(angles[1])),self.Rx(angles[2]))
        else:
            print("Wrong order, provide in using 'xyz'")

    def Rx(self,phi):
        return np.array([[1, 0, 0],
                     [0, np.cos(phi), -np.sin(phi)],
                     [0, np.sin(phi), np.cos(phi)]])

    def Ry(self,theta):
        return np.array([[np.cos(theta), 0, np.sin(theta)],
                         [0, 1, 0],
                         [-np.sin(theta), 0, np.cos(theta)]])

    def Rz(self, psi):
        return np.array([[np.cos(psi), -np.sin(psi), 0],
                 [np.sin(psi), np.cos(psi), 0],
                 [0, 0, 1]])



def plot_3d_rot(ax, rotation_mat=None, translation=[0,0.05,0]):
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

    p4 = translation

    p1 = (np.matmul(np.array(rotation_mat), np.array([[1,0,0]]).T) + np.array([translation]).T).T[0]
    print(p1)
    p2 =  (np.matmul(np.array(rotation_mat), np.array([[0,1,0]]).T) + np.array([translation]).T).T[0]
    p3 =  (np.matmul(np.array(rotation_mat), np.array([[0,0,1]]).T) + np.array([translation]).T).T[0]

    ax.plot([p4[0],p1[0]],[p4[1],p1[1]],[p4[2],p1[2]], color="r")
    ax.plot([p4[0],p2[0]],[p4[1],p2[1]],[p4[2],p2[2]], color="g")
    ax.plot([p4[0],p3[0]],[p4[1],p3[1]],[p4[2],p3[2]], color="b")



if __name__=="__main__" :
    
    rotMat = np.array([[0,-1,0],[0,0,-1],[1,0,0]], dtype=np.float64)
    rotMat_cam0 = np.array([[ 6.71487041e-01,  7.41014515e-01,  1.62525572e-03],
 [-7.41009773e-01,  6.71488718e-01, -2.72375351e-03],
 [-3.10968177e-03,  6.24634817e-04,  9.99994970e-01]], dtype=np.float64)


    t_cam0 = [0.0,0.0,0.0]



    rotMat_cam1 = np.array([[-0.999991980508952, 0.0032425627112567875, -0.002350469069725772 ],
                            [0.0023816581389216724, 0.009648555620498319, -0.9999506153200506 ],
                            [-0.0032197239467815653, -0.9999481942388326, -0.009656200919865136]], dtype=np.float64)

    t_cam1= [-0.05880865244539242, -0.1215735034443526, -0.0856024019820022]

    #q1 = Quaternion(rotMat_cam0)
    #rot_mat = Rotations([180,0,45], extrinsic=True).rotation_matrix
    #q1 = Quaternion( [-1,0.0,0.0,0.0])
    q2 = Quaternion([0.0, 45,45], deg=True) 
    #q3 = Quaternion([0.0, 30.0, 0.0], deg=True)
    #print(q2)
    #print(q1.inv().rotate(q1.rotate([0,0,0.5]).to_list()[:3]).to_list())#.to_euler(deg=True))
    #print(q2.to_rvec())
    #print((q1*q3).rotate([0,0,-1]).to_list())

    fig, ax = plt.subplots(1)
    rear_lidar_rot = [[-0.9838728439172247, -0.03489949670250109, -0.17543161865701454],
            [0.03435759679162382, -0.9993908270190958, 0.0061262071166857945],
            [-0.175538552, -2.14972725e-17, 0.984472558]] 
    rear_camera_rot = [[-0.02472500240085318, 0.9993908269999907, 0.02463024897463667], 
            [0.7080314512806554, 0.03489949724959929, -0.7053180056464845],
            [-0.7057479282673715, 1.5439415856910443e-09, -0.7084630277906626]] 


    # q1 = Quaternion(rear_lidar_rot)#Quaternion( [-0.0880986,0.0015378,0.9959589,0.0173845] )
    # q2 = Quaternion(rear_camera_rot)#Quaternion( [0.6420355,0.6648473,-0.2652181,0.2746413] )
    # q3 = Quaternion( [-0.21263111527614184,-0.21263111527614184,0.6743797400305046,0.6743797063827515] )
    # q4 = Quaternion( [0.21076748742763302,-0.21447855340451885,-0.6684690676326347, 0.6802390217781067] )

    t1 = [-1.6602861245985938, 0.0,1.732975221562235]
    t2 = [ -1.6532861245985937, 0.0, 1.6729752215622349]
    t3 = [ -0.11797776996071918, 1.0, 2.5067800929719004]
    t4 = [-0.13349286393377516, -1.0149782312015498, 2.5193396640847534]

    #print(q1.to_euler(deg=True),q2.to_euler(deg=True))

    plot_3d_rot(ax, rear_camera_rot, t2)
    plt.show()
    
    #plot_3d(ax, q2, t2, ["rear_camera"])
    plot_3d(ax, q2)
    #plot_3d(ax, (q1*q3).rotate([0,0,-1]))
    plt.show()
    
"""
[[ 6.71487041e-01,  7.41014515e-01,  1.62525572e-03],
 [-7.41009773e-01,  6.71488718e-01, -2.72375351e-03],
 [-3.10968177e-03,  6.24634817e-04,  9.99994970e-01]]
"""

"""

- position:
    [-1.6602861245985938, 0.0,1.732975221562235]
  orientation:
    [-0.0880986,0.0015378,0.9959589,0.0173845]
- position:
    [ -1.6532861245985937, 0.0, 1.6729752215622349]
  orientation:
    [0.6420355,0.6648473,-0.2652181,0.2746413]
- position:
    [ -0.11797776996071918, 1.0, 2.5067800929719004]
- orientation:
    [-0.21263111527614184,-0.21263111527614184,0.6743797400305046,0.6743797063827515]
- position:
    [-0.13349286393377516, -1.0149782312015498, 2.5193396640847534]
  orientation:
    [0.21076748742763302,-0.21447855340451885,-0.6684690676326347, 0.6802390217781067]
"""

