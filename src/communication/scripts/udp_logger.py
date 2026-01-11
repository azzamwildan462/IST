#!/usr/bin/python3

import rclpy
from rclpy.node import Node 

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray

from loguru import logger

# udp utils 
import sys
import os
import socket
import struct
from dataclasses import dataclass, asdict, replace
import time
from threading import Lock
import numpy as np

@dataclass
class towing_t:
    name: int = -1
    soc: int = 0
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_theta: float = 0.0
    terminal: int = -1
    warning: int = 0
    lap: int = 0
    ts_ms: int = 0

class UDPLOGGER(Node):
    def __init__(self):
        super().__init__('udp_logger_node')
        self.get_logger().info("UDPLOGGER Node has been started.")

        # Logger
        # ------
        logger.remove()
        logger.add(
            sys.stdout,
            colorize=True,
            format="<green>{time:HH:mm:ss.SSS}</green> | <level>{level:^6}</level> | <cyan>{function}</cyan>:<cyan>{line}</cyan> - {message}",
            enqueue=True,
        )

        self.declare_parameter("T_ID", -1)
        self.declare_parameter("MY_SERVER_IP", "0.0.0.0")
        self.declare_parameter("MY_SERVER_PORT", 1254)
        self.declare_parameter("T2_IP", "10.20.30.40")
        self.declare_parameter("T2_PORT", 1255)
        self.declare_parameter("OFC_IP", "10.20.30.245")
        self.declare_parameter("OFC_PORT", 1255)
        self.declare_parameter("JB_IP", "10.20.30.221")
        self.declare_parameter("JB_PORT", 1254)
        self.declare_parameter("MODE_DEBUG", 0)
        self.declare_parameter("TIMEOUT_TO_RESTART_S", 30)
        self.declare_parameter("TIMEOUT_NOT_RECV_SERVER", 30)
        self.declare_parameter("ENABLE_WATCHDOG", 1)

        self.T_ID = self.get_parameter("T_ID").get_parameter_value().integer_value
        self.MY_SERVER_IP = self.get_parameter("MY_SERVER_IP").get_parameter_value().string_value
        self.MY_SERVER_PORT = self.get_parameter("MY_SERVER_PORT").get_parameter_value().integer_value
        self.T2_IP = self.get_parameter("T2_IP").get_parameter_value().string_value
        self.T2_PORT = self.get_parameter("T2_PORT").get_parameter_value().integer_value
        self.OFC_IP = self.get_parameter("OFC_IP").get_parameter_value().string_value
        self.OFC_PORT = self.get_parameter("OFC_PORT").get_parameter_value().integer_value
        self.JB_IP = self.get_parameter("JB_IP").get_parameter_value().string_value
        self.JB_PORT = self.get_parameter("JB_PORT").get_parameter_value().integer_value
        self.MODE_DEBUG = self.get_parameter("MODE_DEBUG").get_parameter_value().integer_value
        self.TIMEOUT_TO_RESTART_S = self.get_parameter("TIMEOUT_TO_RESTART_S").get_parameter_value().integer_value
        self.TIMEOUT_NOT_RECV_SERVER = self.get_parameter("TIMEOUT_NOT_RECV_SERVER").get_parameter_value().integer_value
        self.ENABLE_WATCHDOG = self.get_parameter("ENABLE_WATCHDOG").get_parameter_value().integer_value


        # Record last time to start 
        self.last_time_started = time.time()
        self.last_pose_x_by_server = 0.0
        self.last_pose_y_by_server = 0.0
        self.last_time_update_from_server = time.time()

         # Thread lock

        self.lock = Lock()

        self.tsaya = towing_t()
        self.tsaya.name = self.T_ID

        self.info_teman = [];

        self.udp_init_as_server()
        self.udp_init_as_client()

        # Subscriber
        # ----------
        self.sub_odom = self.create_subscription(Odometry, "/master/pose_filtered", self.callback_sub_odom, 1)
        self.sub_status_emergency = self.create_subscription(Int16, "/master/status_emergency", self.callback_sub_status_emergency, 1)
        self.sub_terminal_terakhir = self.create_subscription(Int16, "/master/terminal_terakhir", self.callback_sub_terminal_terakhir, 1)
        self.sub_battery_soc = self.create_subscription(Int16, "/master/battery_soc", self.callback_sub_battery_soc, 1)
        self.sub_counter_lap = self.create_subscription(Int32, "/master/counter_lap", self.callback_sub_counter_lap, 1)

        self.pub_info_teman = self.create_publisher(Float32MultiArray, "/udp/info_teman", 1)

        logger.info(f"UDPLOGGER Initialized")

        self.counter_error = 0
        
        # ROS2 Timer
        # ----------
        self.timer = self.create_timer(1.0, self.timer_callback)

    
    def udp_init_as_server(self):
        try:
            self.sock_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock_server.bind((self.MY_SERVER_IP, self.MY_SERVER_PORT))
            self.sock_server.setblocking(False)
            logger.info(f"UDP server initialized on {self.MY_SERVER_IP}:{self.MY_SERVER_PORT}")
        except OSError as e:
            self.sock_server = None
            logger.warning(f"Failed to init UDP server ({e}); will retry later")
    
    def udp_init_as_client(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.connect((self.JB_IP, self.JB_PORT))  
            sock.setblocking(False)
            self.sock_client_jb = sock
            logger.info(f"UDP client initialized to send to {self.JB_IP}:{self.JB_PORT}")
        except OSError as e:
            self.sock_client_jb = None
            logger.warning(f"Failed to init UDP client ({e}); will retry later")


    def callback_sub_odom(self, msg):
        self.lock.acquire()
        self.tsaya.pose_x = msg.pose.pose.position.x
        self.tsaya.pose_y = msg.pose.pose.position.y
        self.tsaya.pose_theta = np.arctan2(2*(msg.pose.pose.orientation.w*msg.pose.pose.orientation.z), 1-2*(msg.pose.pose.orientation.z**2))
        self.lock.release()
    
    def callback_sub_status_emergency(self, msg):
        self.lock.acquire()
        self.tsaya.warning = msg.data
        self.lock.release()

    def callback_sub_terminal_terakhir(self, msg):
        self.lock.acquire()
        self.tsaya.terminal = msg.data
        self.lock.release()

    def callback_sub_battery_soc(self, msg):
        self.lock.acquire()
        self.tsaya.soc = msg.data
        self.lock.release()

    def callback_sub_counter_lap(self, msg):
        self.lock.acquire()
        self.tsaya.lap = msg.data
        self.lock.release()

    def timer_callback(self):
        
        # Khusus tes
        if self.MODE_DEBUG == 1:
            self.tsaya.soc = 89
            self.tsaya.warning = 1
            self.tsaya.terminal = 5
            self.tsaya.lap += 1

            if self.tsaya.lap > 10:
                self.tsaya.lap = 0


        self.lock.acquire()
        try:
            packed_data = struct.pack(
                'i i f f f i i i Q',
                self.tsaya.name,
                self.tsaya.soc,
                self.tsaya.pose_x,
                self.tsaya.pose_y,
                self.tsaya.pose_theta,
                self.tsaya.terminal,
                self.tsaya.warning,
                self.tsaya.lap,
                int(time.time_ns() // 1_000_000),
            )
            if self.sock_client_jb:
                self.sock_client_jb.send(packed_data)
                logger.info(f"Sent data to {self.JB_IP}:{self.JB_PORT}: {asdict(self.tsaya)}")
                self.counter_error = 0
        except (BlockingIOError, InterruptedError):
            pass
        except Exception as e:
            logger.error(f"send {self.JB_IP} error: {e}")

            # if e.errno == 101:
            self.counter_error += 1
            if self.counter_error >= 20:
                # logger.warning(f"Re-initializing UDP client to {self.JB_IP}:{self.JB_PORT}")

                if getattr(self, "sock_client_jb", None):
                    try:
                        self.sock_client_jb.close()
                    except Exception:
                        pass
                self.udp_init_as_client()
                self.counter_error = 0
        self.lock.release()

        # Receive data from server
        if self.sock_server:
            try:
                while True:
                    data, addr = self.sock_server.recvfrom(1024, socket.MSG_DONTWAIT)
                    if data:
                        float_array = struct.unpack('f f f f f f f f f f f f', data)
                        msg = Float32MultiArray()
                        msg.data = float_array
                        self.pub_info_teman.publish(msg)

                        self.last_time_update_from_server = time.time()
                        if self.T_ID == 1:
                            self.last_pose_x_by_server = float_array[2]
                            self.last_pose_y_by_server = float_array[3]
                        elif self.T_ID == 2:
                            self.last_pose_x_by_server = float_array[5]
                            self.last_pose_y_by_server = float_array[6]
                        elif self.T_ID == 3:
                            self.last_pose_x_by_server = float_array[9]
                            self.last_pose_y_by_server = float_array[10]

                        # logger.info(f"Received data from {addr}: {float_array}")
            except (BlockingIOError, InterruptedError):
                pass
            except Exception as e:
                logger.error(f"receive error: {e}")
        
        # Watchdog 
        if self.ENABLE_WATCHDOG == 1:
            # WAtchdog aktif ketika melebihi TIMEOUT_TO_RESTART_S
            if time.time() - self.last_time_started > self.TIMEOUT_TO_RESTART_S:

                # Jika gabisa menerima dari server
                if time.time() - self.last_time_update_from_server > self.TIMEOUT_NOT_RECV_SERVER:
                    logger.warning("No update from server for a while. Restarting node...")
                    rclpy.shutdown()
                
                # Jika bisa menerima dari server, tapi gabisa ngirim ke server
                if self.last_pose_x_by_server > 99998.0 and self.last_pose_y_by_server > 99998.0:
                    logger.warning("Position from server is invalid. Restarting node...")
                    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node_udp_logger = UDPLOGGER()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_udp_logger)
    executor.spin()
    
if __name__ == '__main__':
    main(sys.argv)