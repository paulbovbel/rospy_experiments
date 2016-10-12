#!/usr/bin/env python
from functools import partial
from pympler import tracker
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

import gc
import random
import rospy
import string
import threading
import progressbar
import time


def service_cb(req, event):
    event.set()
    return SetBoolResponse()


def run_test():
    objects = []
    samples = 1000
    bar = progressbar.ProgressBar(max_value=samples)

    print("Creating, testing and destroying {} services").format(samples)

    for idx in range(samples):
        bar.update(idx/2)
        name = ''.join(random.choice(string.ascii_letters) for _ in range(10))
        event = threading.Event()
        client = rospy.ServiceProxy(name, SetBool)
        service = rospy.Service(name, SetBool, partial(service_cb, event=event))
        objects.append((idx, client, service, event))

    for idx, client, service, event in objects:
        bar.update((idx + samples)/2)
        client.wait_for_service()
        client.call(SetBoolRequest())
        event.wait()

        service.shutdown()
        client.close()


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
