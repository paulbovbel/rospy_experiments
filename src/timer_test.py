#!/usr/bin/env python
from functools import partial
from pympler import tracker
from std_msgs.msg import String

import gc
import random
import rospy
import string
import threading
import progressbar
import time


def event_callback(event, threading_event):
    threading_event.set()


def run_test():
    timers = []
    samples = 10000
    bar = progressbar.ProgressBar(max_value=samples)

    print("Creating, testing and destroying {} timers").format(samples)

    for idx in range(samples):
        bar.update(idx / 2)
        event = threading.Event()
        timer = rospy.Timer(
            rospy.Duration(0.1), partial(event_callback, threading_event=event), oneshot=True)
        timer.name = "Timer{}".format(idx)
        timers.append((idx, timer, event))

    for idx, timer, event in timers:
        bar.update((idx + samples)/2)
        event.wait()
        timer.shutdown()
        while timer.is_alive():
            time.sleep(0.01)

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
    print("Active threads: {}").format(
        ','.join([thread.name for thread in threading.enumerate()]))
