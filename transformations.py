import numpy as np
import matplotlib.pyplot as plt
import math 




def euler_to_quaternion():
    pass

def view_quaternion_as_axes():
    pass

def euler_to_rotation_matrix():
    pass

def quaternion_to_rotation_matrix():
    pass



class Quaternion:

    def __init__(self,x,y,z,w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        self.magnitude = math.sqrt(x**2 + y**2 + z**2 + w**2)
    
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
        return Quaternion(self.x,self.y,self.z,self.w)

    def divide(self, scalar):
        assert not isinstance(scalar, Quaternion), "can only divide scalars"
        return Quaternion(self.x/scalar, self.y/scalar, self.z/scalar, self.w/scalar)
    
    def conjugate(self):
        return Quaternion(-self.x, -self.y, -self.z, self.w)
    
    def inv(self):
        return self.conjugate().divide(self.magnitude)


q =Quaternion(1,1,1,1)
print(q.inv().magnitude) 
    
    

    
    



