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


def run_test():
    pubs = []
    samples = 1000
    bar = progressbar.ProgressBar(max_value=samples)

    print("Creating and destroying {} publishers").format(samples)

    for idx in range(samples):
        bar.update(idx/2)
        topic = ''.join(random.choice(string.ascii_letters) for _ in range(10))
        pub = rospy.Publisher(topic, String, queue_size=10)
        pub.publish(String(data="asdf"))
        pubs.append(pub)

    for idx, pub in enumerate(pubs):
        bar.update((idx + samples)/2)
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
