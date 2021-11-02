import rosbag
import rospy
import numpy as np
import cv2
import sensor_msgs.point_cloud2 as pc2
import subprocess
import matplotlib.pyplot as plt
from cv_bridge import CvBridge

def time_check(rostime1, rostime2, tolerance):
    return abs(rostime1.to_sec() - rostime2.to_sec()) < tolerance

def rosmsg_to_list(bag, camera_left_topic, camera_right_topic, lidar_topic="/os_cloud_node/points"):
    cam_left=[]
    cam_right=[]
    lidar=[]
    for topic, msg, t in bag.read_messages():
        if topic == camera_left_topic:
            cam_left.append(msg)
        elif topic == camera_right_topic:
            cam_right.append(msg)
        elif topic == lidar_topic:
            lidar.append(msg)
    return cam_left, cam_right, lidar

def rosmsg_to_dict(msg_list):
    msg_dict={}
    for i in msg_list:
        msg_dict[i.header.stamp.to_sec()] = i
    return msg_dict

def time_checker(time, time_list, offset1, offset2):
    time_list = np.array(time_list) + offset2
    diff= np.array([abs((time.to_sec()+offset1 -i)) for i in time_list])
    return np.min(diff), np.argmin(diff)
        







# Some parameters
name="Recording_2021-02-10_16-33-04_hyslam_demo"
folder = "/home/hyslam/Datasets/" 
# image_folder = "/SensingXavierB/Logger/Video1/"

image_points =[]
synced_imgs=[]

# Extrinsic matrix
camera_matrix =  np.array([[-0.99959308,  0.02818052, -0.00442073, 0.06508126],
                            [ 0.00408373, -0.0120047,  -0.9999196,  -0.12425698],
                            [-0.02823132 ,-0.99953076,  0.01188474, -0.08094532]])
# Instrisic matrix 
mat = np.array([[965.044522822366, 0,497.18057271858487],
                [0, 965.8373876276721,312.42332843193617],
                [0, 0, 1]])

dist_coeffs = np.array([-0.3185388316275337, 0.1467501559423749, 0.002890954216910735,0.0004298879783934694])




# Ros bagdata
bag = rosbag.Bag(folder+ name +"/ROS1/"+ name +".bag")
cam_msgs, cam_right_msgs, lidar_msgs = rosmsg_to_list(bag, "/gmsl_video1_ros1/image_raw", "/gmsl_video2_ros1/image_raw")
cam_dict = rosmsg_to_dict(cam_msgs)


if (len(cam_msgs) == len(cam_right_msgs)):
    print("[Camera] left and right have same msgs")
    cnt = 0
    for i, ms in enumerate(cam_msgs):
        if time_check(ms.header.stamp, cam_right_msgs[i].header.stamp, 1e-5):
            cnt +=1
    assert cnt ==len(cam_msgs), "[Camera] Left and right have no frame drops but out of sync"
else:
    print("[Camera] Few frames from the camera have been lost")
    cnt = 0
    for msg in cam_right_msgs:
        diff, index = time_checker(msg.header.stamp, cam_dict.keys(), 0.0,0.0)
        if diff < 1e-5:
            cnt += 1
    assert cnt == len(cam_right_msgs), "[Camera] Not all left and right camera stamps  are synced"
        



X_MAX = 8
Y_MAX = 40
X_MIN = -10
Y_MIN = -10

fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.set_title("Lidar Overview")
ax.set_xlim([X_MIN, X_MAX])
ax.set_ylim([Y_MIN, Y_MAX])
ax.set_autoscale_on(False)
sct = None
# sync of frames to lidar
cnt = 0
for msg in lidar_msgs:
    diff, index = time_checker(msg.header.stamp, cam_dict.keys(), 0.0,0.0)#0.211235
    if diff < 0.033:
        cnt+=1
        synced_imgs.append(cam_dict[cam_dict.keys()[index]])
        points = np.array([[p[0],p[1],p[2], 1] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True) if -p[1]> 0], dtype=np.float64) # neglect all rear points to camera
        image_points.append(np.array([np.matmul(mat,np.matmul(camera_matrix, point.T)) for point in points]))
        if sct is not None:
            sct.remove()
        sct = ax.scatter(-points[:,0], -points[:,1], s=0.2, c="b")
        fig.canvas.draw() 
        plt.pause(0.05)
print("[Lidar] Number of frames lost : ", float(cnt - len(lidar_msgs))/ float(len(lidar_msgs)))


bridge = CvBridge()
dim = bridge.imgmsg_to_cv2(synced_imgs[0], desired_encoding='bgr8').shape
print(dim)
# Video storing
f = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter("output_sync_check.avi",f, 30, (dim[1],dim[0]))
# Project lidar points on to the image     
for j,image_s in enumerate(image_points):
    image_bounds = image_s[np.where(np.logical_and( np.logical_and(image_s[:,0] > 0 , image_s[:,0] < dim[1]), np.logical_and(image_s[:,1] > 0 , image_s[:, 1] < dim[0])))]
    blank_image =  bridge.imgmsg_to_cv2(synced_imgs[j], desired_encoding="bgr8") #np.zeros(dim, dtype=np.uint8)
    if len(image_bounds) > 1:
        max_z = np.max(image_bounds[:,2])
        for i in image_bounds:
            cv2.drawMarker(blank_image, (int(i[0]), int(i[1])),(0, 0 ,int(i[2]/max_z * 255)), markerType=cv2.MARKER_STAR, markerSize=3, thickness=2, line_type=cv2.LINE_AA)
            cv2.imshow("",blank_image)
            out.write(blank_image)
            cv2.waitKey(1)

out.release()
cv2.destroyAllWindows()

