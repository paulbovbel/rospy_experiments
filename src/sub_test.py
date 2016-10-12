#!/usr/bin/env python
from pympler import tracker
import gc
import rospy
import random
import time
import string
import threading
import progressbar
from std_msgs.msg import String


def sub_callback(data):
    print(data.data)


def run_test():
    subs = []
    samples = 10000
    bar = progressbar.ProgressBar(max_value=samples)

    print("Creating and destroying {} subscribers").format(samples)

    for idx in range(samples):
        bar.update(idx/2)
        topic = ''.join(random.choice(string.ascii_letters) for _ in range(10))
        subs.append(rospy.Subscriber(topic, String, sub_callback))

    for idx, sub in enumerate(subs):
        bar.update((idx + samples)/2)
        sub.unregister()


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
