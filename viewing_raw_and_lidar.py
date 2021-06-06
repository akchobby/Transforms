import rosbag
import rospy
import numpy as np
import cv2
import sensor_msgs.point_cloud2 as pc2
import subprocess
import matplotlib.pyplot as plt

def raw_to_cv(img_name):
    rows = 604
    cols = 960
    fd = open(img_name)
    f = np.fromfile(fd, dtype=np.uint8,count=rows*cols*4)
    im = f.reshape((rows, cols, 4))
    fd.close()
    return cv2.cvtColor(im, cv2.COLOR_RGB2BGR)


def time_checker(time, time_list, offset1, offset2):
    time_list = time_list + offset2
    diff= np.array([abs((time.to_sec()-offset1 -i)) for i in time_list])
    return np.min(diff), np.argmin(diff)

def frequency_check(stamp_list,freq_ms):
    cnt=0
    for i,stamp in enumerate(stamp_list ):
        if np.isclose(abs(stamp_list[i-1].to_sec() - stamp.to_sec()), freq_ms, atol=0.001) :
            cnt +=1
    return cnt

def restamp_rosbag(bag_name, topic_list, offset):
    with rosbag.Bag(bag_name + "_restamped_lidar_utc_offset.bag", 'w') as outbag:
        for topic, msg, t in rosbag.Bag(bag_name + ".bag").read_messages():
            if topic in topic_list:
                msg.header.stamp -= offset
            outbag.write(topic, msg, t)
            

def tai_utc_offset_rosbag(bag_name, topic_list=["/os_cloud_node/points", "/os_cloud_node/imu" ] ,on_system=False):
    if not on_system:
        utc_offset = rospy.Duration.from_sec(36) # hard coded value based on current ptp setup
        restamp_rosbag(bag_name, topic_list, utc_offset)
    else:
        lines = subprocess.Popen(["sudo pmc -u -b 0 'get TIME_PROPERTIES_DATA_SET' | grep currentUtc"],shell=True, stdout=subprocess.PIPE).communicate()[0]
        lines_cleaned=[line.strip().split() for line in lines.split("\n")]
        if len(lines_cleaned[0]) > 1:
            if int(lines_cleaned[1][1]) == 1:
                print("[WARN]: ptp protocol using valid offset, adviced not to add offset to rosbag")
            utc_offset = rospy.Duration.from_sec(int(lines_cleaned[0][1]))
            restamp_rosbag(bag_name, topic_list, utc_offset)
        else:
            print("[ERROR]: ptp clock not running")


# Some parameters
name="Recording_2021-05-21_13-59-54_hyslam_recording_ros1_lidar_ros2_position_files_caml_high_speed"
folder = "/home/hyslam/Downloads/" 
image_folder = "/SensingXavierB/Logger/Video1/"

lidar_stamps = []
lidar_msgs = []
camera_stamps = []
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


# Lidar Transforms
trans_lidar_imu = [0.006, -0.012, -0.029] 
quat_lidar_imu = [0.000, 0.000, -1.000, 0.000]

# Ros bagdata
bag = rosbag.Bag(folder+ name +"/ROS1/"+ name +".bag")
for topic, msg, t in bag.read_messages(topics=["/os_cloud_node/points"]):
    lidar_stamps.append(msg.header.stamp)
    lidar_msgs.append(msg)
bag.close()


# file reading
with open(folder+name+image_folder+"timestamp.txt",'ro') as file:
    img_files = file.readlines()


# Reading data 
img_names=[]
for i in img_files:
    img_names.append(folder+name+image_folder+i.split(" ")[3].strip())
    camera_stamps.append(np.float64(i.split(" ")[1].strip())/1000000.0)
    #undistorted_image = cv2.undistort(disp, camera_matrix, dist_coeffs, None)



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
for i,lidar_msg in enumerate(lidar_msgs):
    diff, index = time_checker(lidar_stamps[i], np.array(camera_stamps), 0.0, 0.0) #36, 0.211236355189
    if diff < 0.013:
        synced_imgs.append(img_names[index])
        points = np.array([[p[0],p[1],p[2], 1] for p in pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True) if -p[1]> 0], dtype=np.float64) # neglect all rear points to camera
        image_points.append(np.array([np.matmul(mat,np.matmul(camera_matrix, point.T)) for point in points]))

        if sct is not None:
            sct.remove()
        sct = ax.scatter(-points[:,0], -points[:,1], s=0.2, c="b")
        fig.canvas.draw() 
        plt.pause(0.05)


dim = raw_to_cv(img_names[0]).shape
# Video storing
f = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter("output_sync_check.avi",f, 10, (dim[1],dim[0]))
# Project lidar points on to the image     
for j,image_s in enumerate(image_points):
    image_bounds = image_s[np.where(np.logical_and( np.logical_and(image_s[:,0] > 0 , image_s[:,0] < dim[1]), np.logical_and(image_s[:,1] > 0 , image_s[:, 1] < dim[0])))]
    blank_image =  cv2.undistort(raw_to_cv(synced_imgs[j]), mat, dist_coeffs, None) #np.zeros(dim, dtype=np.uint8)
    max_z = np.max(image_bounds[:,2])
    for i in image_bounds:
        cv2.drawMarker(blank_image, (int(i[0]), int(i[1])),(0, 0 ,int(i[2]/max_z * 255)), markerType=cv2.MARKER_STAR, markerSize=3, thickness=2, line_type=cv2.LINE_AA)
    cv2.imshow("",blank_image)
    out.write(blank_image)
    cv2.waitKey(1)

out.release()
cv2.destroyAllWindows()

"""
 Calibration data
 cam0:
  T_cam_imu:
  [[-0.9995930750515504, 0.028180516203601146, -0.004420725673603042, 0.06508125947885363]
  [0.00408373330785272, -0.012004702762037862, -0.9999196018850028, -0.12425698102638041]
  [-0.028231320040922887, -0.9995307627172317, 0.01188473603633216, -0.08094531518938053]
  [0.0, 0.0, 0.0, 1.0]]
  cam_overlaps: [1]
  camera_model: pinhole
  distortion_coeffs: [-0.3185388316275337, 0.1467501559423749, 0.002890954216910735,
    0.0004298879783934694]
  distortion_model: radtan
  intrinsics: [965.044522822366, 965.8373876276721, 497.18057271858487, 312.42332843193617]
  resolution: [960, 604]
  rostopic: /gmsl_video1/image_raw
np.array([[p[0],p[1],p[2]] for p in pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)], dtype=np.float64)

T_ci:  (imu0 to cam0): 
[[-0.99959308  0.02818052 -0.00442073  0.06508126]
 [ 0.00408373 -0.0120047  -0.9999196  -0.12425698]
 [-0.02823132 -0.99953076  0.01188474 -0.08094532]
 [ 0.          0.          0.          1.        ]]

T_ic:  (cam0 to imu0): 
[[-0.99959308  0.00408373 -0.02823132  0.06327702]
 [ 0.02818052 -0.0120047  -0.99953076 -0.08423302]
 [-0.00442073 -0.9999196   0.01188474 -0.12299727]
 [ 0.          0.          0.          1.        ]]
"""

print("completed")