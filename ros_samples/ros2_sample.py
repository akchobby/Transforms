import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String
from multiprocessing import Lock
from common_class import CommonClass


class MinimalSubscriber(Node):

    def __init__(self, common_lock, common_variable):
        super().__init__('minimal_sub')
        self.subscription = self.create_subscription(String,'topic',self.listener_callback,10)
        self.mutex = common_lock
        self.filter = common_variable
        self.i = 0


    def listener_callback(self, msg):
        self.mutex.acquire()
        #print(f"subscriber got mutex {self.i}\n")
        self.filter.update(msg)
        self.mutex.release()
        self.i += 1

class MinimalPublisher(Node):

    def __init__(self, common_lock, common_variable):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.mutex = common_lock
        self.common_variable = common_variable

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.mutex.acquire()
        #print(f"publisher got mutex {self.i}")
        self.common_variable.update(self.i)
        self.mutex.release()
        self.publisher_.publish(msg)
        self.i += 1

class GrandParent(Node):

    def __init__(self, s="default"):
        super().__init__("class_inheritance_play")
        self.name = s
        


class Parent(GrandParent):

    def __init__(self):
        super().__init__("Parent")
        self.age = 30
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        print("from Parent")


class Child(Parent):

    def __init__(self):
        super().__init__()
    
    def timer_callback(self):
        print("from Child")


def main(args=None):


    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    cla = Child()
    executor.add_node(cla)
    executor.spin()
    executor.shutdown()

    # Multithreaded Example
    # rclpy.init()
    # executor = rclpy.executors.MultiThreadedExecutor(2)
    # mutex = Lock()
    # common_var = CommonClass()

    # minimal_publisher = MinimalPublisher(mutex, common_var)
    # minimal_subscriber = MinimalSubscriber(mutex, common_var)
    # print(common_var.msg)
    # executor.add_node(minimal_publisher)
    # executor.add_node(minimal_subscriber)

    # executor.spin()
    # executor.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)


if __name__ == '__main__':
    main()
