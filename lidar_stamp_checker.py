import rosbag
import rospy
import numpy as np
import subprocess
import sys

def rosmsg_to_list_stamps(bag, camera_left_topic, lidar_topic="/os_cloud_node/points"):
    cam_left = [msg.header.stamp.to_sec() for topic, msg, t in bag.read_messages()  if topic == camera_left_topic]
    lidar = [msg.header.stamp.to_sec() for topic, msg, t in bag.read_messages()  if topic == lidar_topic]
    return cam_left, lidar


def time_checker(time, time_list):
    return np.min([abs((time-i)) for i in time_list])

def restamp_rosbag(bag_name, topic_list, offset):
    with rosbag.Bag(bag_name + "_restamped_lidar_utc_offset.bag", 'w') as outbag:
        for topic, msg, t in rosbag.Bag(bag_name + ".bag").read_messages():
            if topic in topic_list:
                msg.header.stamp -= offset
            outbag.write(topic, msg, t)
            

def tai_utc_offset_rosbag(bag_name, topic_list=["/os_cloud_node/points", "/os_cloud_node/imu" ] ,on_system=False):
    if not on_system:
        utc_offset = rospy.Duration.from_sec(36) # hard coded value based on current ptp setup 36
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
    folder = sys.argv[1] #"/home/hyslam/Datasets/" 
    name= sys.argv[2]  #"Recording_2021-02-10_16-33-04_hyslam_demo"
    system_flag = bool(int(sys.argv[3]))

    # Ros bagdata
    bag = rosbag.Bag(folder+ name +"/ROS1/"+ name +".bag")
    cam_msgs, lidar_unstamped = rosmsg_to_list_stamps(bag, "/gmsl_video1_ros1/image_raw")

    # First Check
    cnt=0 
    for stamp in lidar_unstamped:
        diff = time_checker(stamp, cam_msgs) #0.211235
        if diff < 0.013:
            cnt+=1
    
    #  Restamping if needed
    if (float(cnt)/float(len(lidar_unstamped))) < 0.9:
        del lidar_unstamped[:] 
        print("[Lidar] More than 10per cent out of sync, restamping ...")
        tai_utc_offset_rosbag(folder+ name +"/ROS1/"+ name, on_system=system_flag)

        bag_restamped = rosbag.Bag(folder+ name +"/ROS1/"+ name +"_restamped_lidar_utc_offset.bag")
        _, lidar_stamped = rosmsg_to_list_stamps(bag_restamped, None)

        cnt=0
        for stamp in lidar_stamped:
            diff = time_checker(stamp, cam_msgs)
            if diff < 0.013:
                cnt+=1
        
        assert float(cnt)/float(len(lidar_stamped)) > 0.95, "[Lidar] Failed! after offset bag not synced, sync_cnt:{} msgs_cnt:{}".format(cnt, len(lidar_stamped))
        print("[Lidar] Restamp sucessful :)")
    print("[Lidar] Stamp check done")

if __name__=="__main__":
    main()