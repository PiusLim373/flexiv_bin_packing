#!/usr/bin/python3
#! /usr/bin/env python3

import threading
import typing

import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
import numpy.typing as npt
import rclpy
from rclpy.subscription import Subscription
from rclpy.node import Node
import random
from std_msgs.msg import Int16
from std_srvs.srv import Trigger
from mpl_toolkits.mplot3d import Axes3D


class Example_Node(Node):
    """Example Node for showing how to use matplotlib within ros 2 node
    
    Attributes:
        fig: Figure object for matplotlib
        ax: Axes object for matplotlib
        x: x values for matplotlib
        y: y values for matplotlib
        lock: lock for threading
        _sub: Subscriber for node
    """

    def __init__(self):
        """Initialize."""
        super().__init__("example_node")
        # Initialize figure and axes and save to class
        self.fig, self.ax = plt.subplots()
        # create Thread lock to prevent multiaccess threading errors
        self._lock = threading.Lock()
        # create initial values to plot
        self.x = [i for i in range(5)]
        self.width = [i for i in range(5)]
        # create subscriber
        self.cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._sub: Subscription = self.create_subscription(
            Int16, "test", self._callback, 10, callback_group=self.cbg
        )
        self.srv = self.create_service(Trigger, "pack", self.service_cb)
    
    def service_cb(self, req, res):
        self.x.append(random.randint(1, 10))
        self.width.append(random.randint(1, 10))
        res.success = True
        return res
    
    def _callback(self, msg):
        """Callback for subscriber
        
        Args:
            msg: message from subscriber
                Message format
                ----------------
                int32 num
        """
        # lock thread
        with self._lock:
            # update values
            self.x.append(random.randint(0, 10))
            self.width.append(random.randint(0, 10))

    def plt_func(self, _):
        """Function for for adding data to axis.

        Args:
            _ : Dummy variable that is required for matplotlib animation.
        
        Returns:
            Axes object for matplotlib
        """
        # lock thread
        print("hellow fron pltfunc")
        with self._lock:
            self.ax.plot(self.x, self.width, label="Random Data")
            

            return self.ax

    def _plt(self):
        """Function for initializing and showing matplotlib animation."""
        self.ani = anim.FuncAnimation(self.fig, self.plt_func, interval=1000)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = Example_Node()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    node._plt()


if __name__ == "__main__":
    main()
