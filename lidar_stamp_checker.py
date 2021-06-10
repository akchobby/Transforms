import rosbag
import rospy
import numpy as np
import subprocess
import sys

def rosmsg_to_list_stamps(bag, camera_left_topic, camera_right_topic, lidar_topic="/os_cloud_node/points"):
    cam_left=[]
    cam_right=[]
    lidar=[]
    for topic, msg, t in bag.read_messages():
        if topic == camera_left_topic:
            cam_left.append(msg.header.stamp.to_sec())
        elif topic == camera_right_topic:
            cam_right.append(msg.header.stamp.to_sec())
        elif topic == lidar_topic:
            lidar.append(msg.header.stamp.to_sec())
    return cam_left, cam_right, lidar


def time_checker(time, time_list, offset1, offset2):
    time_list = np.array(time_list) + offset2
    diff= np.array([abs((time+offset1 -i)) for i in time_list])
    return np.min(diff), np.argmin(diff)

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



def main():

    # Some parameters
    name= sys.argv[2]#"Recording_2021-02-10_16-33-04_hyslam_demo"
    folder = sys.argv[1] #"/home/hyslam/Datasets/" 
    system_flag = bool(int(sys.argv[3]))

    # Ros bagdata
    bag = rosbag.Bag(folder+ name +"/ROS1/"+ name +".bag")
    cam_msgs, _, lidar_unstamped = rosmsg_to_list_stamps(bag, "/gmsl_video1_ros1/image_raw", None)

    # First Check
    cnt=0 
    for stamp in lidar_unstamped:
        diff, _ = time_checker(stamp, cam_msgs, 0.0,0.0)#0.211235
        if diff < 0.013:
            cnt+=1
        


    #  Restamping if needed
    if (float(cnt)/float(len(lidar_unstamped))) < 0.9:
        print("[Lidar] More than 10per cent out of sync, restamping")
        tai_utc_offset_rosbag(folder+ name +"/ROS1/"+ name, on_system=system_flag)

        bag_restamped = rosbag.Bag(folder+ name +"/ROS1/"+ name +"_restamped_lidar_utc_offset.bag")
        _, _, lidar_stamped = rosmsg_to_list_stamps(bag_restamped, None, None)

        cnt=0
        for stamp in lidar_stamped:
            diff, _ = time_checker(stamp, cam_msgs, 0.0,0.0)#0.211235
            print(diff)
            if diff < 0.013:
                cnt+=1
        assert float(cnt)/float(len(lidar_unstamped)) > 0.9, "[Lidar] Failed! after offset bag not synced"
        print("Lidar Restamp sucessful :)")
    print("[Lidar] Stamp check done")

if __name__=="__main__":
    main()