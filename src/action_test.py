#!/usr/bin/env python
from functools import partial
from pympler import tracker
from actionlib import msg as actionlib_msg
import actionlib

import gc
import random
import rospy
import string
import threading
import progressbar
import time


class TestServer(object):
    def __init__(self, name):
        self.name = name
        self.server = actionlib.SimpleActionServer(
            name, actionlib_msg.TestAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

    def execute_cb(self, goal):
        result = actionlib_msg.TestResult()
        self.server.set_succeeded(result)

    def shutdown(self):
        pass

class TestClient(object):
    def __init__(self, name):
        self.name = name
        self.client = actionlib.SimpleActionClient(name, actionlib_msg.TestAction)

    def connect(self):
        while True:
            connected = self.client.wait_for_server(rospy.Duration(1.0))
            if connected:
                break
            else:
                print("Waiting to connect to server {}".format(self.name))

    def call(self):
        goal = actionlib_msg.TestGoal()
        self.client.send_goal(goal)
        while True:
            done = self.client.wait_for_result(rospy.Duration(1.0))
            if done:
                break
            else:
                print("Waiting for result from server {}".format(self.name))

    def shutdown(self):
        pass


def run_test():
    objects = []
    samples = 200
    print("Creating, testing, and destroying {} action server and clients").format(samples)
    print("Creating")
    with progressbar.ProgressBar(max_value=samples) as bar:
        for idx in range(samples):
            bar.update(idx)
            action = "test{}".format(idx)
            server = TestServer(action)
            client = TestClient(action)
            objects.append((idx, server, client))

    print("Connecting")
    with progressbar.ProgressBar(max_value=samples) as bar:
        for idx, server, client in objects:
            bar.update(idx)
            client.connect()

    print("Calling")
    with progressbar.ProgressBar(max_value=samples) as bar:
        for idx, server, client in objects:
            bar.update(idx)
            client.call()

    print("Destroying (no-op)")
    with progressbar.ProgressBar(max_value=samples) as bar:
        for idx, server, client in objects:
            bar.update(idx)
            server.shutdown()
            client.shutdown()


if __name__ == '__main__':
    rospy.init_node("test_node")
    tr = tracker.SummaryTracker()

    print("Starting test")
    run_test()
    print("Ended test")

    time.sleep(5)
    gc.collect()

    print("Garbage:")
    tr.print_diff()
    # print("Active threads: {}").format(
    #     ','.join([thread.name for thread in threading.enumerate()]))
