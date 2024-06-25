#!/usr/bin/env python3

import threading
import os
from task_robot import task_robot
from task_llm import task_llm


def task2():
    print("Task 2 assigned to thread: {}".format(threading.current_thread().name))
    print("ID of process running task 2: {}".format(os.getpid()))

if __name__ == "__main__":

    print("ID of process running main program: {}".format(os.getpid()))

    print("Main thread name: {}".format(threading.current_thread().name))
    task_robot.is_true = True

    t1 = task_robot("robot_ctrl")
    t2 = task_llm("llm_app")

    t1.start()
    t2.start()

    t1.join()
    t2.join()
