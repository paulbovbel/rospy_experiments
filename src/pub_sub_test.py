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


def sub_cb(data, event):
    event.set()


def run_test():
    objects = []
    samples = 100
    bar = progressbar.ProgressBar(max_value=samples)

    print("Creating, testing, and destroying {} publishers and subscribers").format(samples)

    for idx in range(samples):
        bar.update(idx/2)
        topic = ''.join(random.choice(string.ascii_letters) for _ in range(10))
        event = threading.Event()
        pub = rospy.Publisher(topic, String, queue_size=10)
        sub = rospy.Subscriber(topic, String, partial(sub_cb, event=event), queue_size=10)
        objects.append((idx, pub, sub, event))

    for idx, pub, sub, event in objects:
        bar.update((idx + samples)/2)
        while not event.wait(0.1):
            pub.publish(String(data="this is dummy data"))

        sub.unregister()
        pub.unregister()


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
