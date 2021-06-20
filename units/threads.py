import time
import threading

# Python Threading Example for Beginners
# First Method
time_sleep = 10


def thread_delay(thread_name, delay):
    count = 0
    while count < 3:
        time.sleep(delay)
        count += 1
        print(thread_name, "-------->", time.time())


# https://realpython.com/intro-to-python-threading/#join-a-thread
t = time.time()

for p in range(0, 20):
    for index, thread in enumerate(threading.enumerate()):
        print(index, thread.name, thread.is_alive())
        if "M1" in thread.name:
            # thread.stop()
            print("stopped " + thread.name)
        if "M2" in thread.name:
            # thread.stop()
            print("stopped " + thread.name)

    # Created the Threads
    t1 = threading.Thread(name="M1", target=thread_delay, args=("T1", time_sleep))
    # Started the threads
    t1.start()
    # print(t1.getName(), t1.is_alive())
    t1 = threading.Thread(name="M2", target=thread_delay, args=("T2", time_sleep / 2))
    t1.start()
    time.sleep(1)

for index, thread in enumerate(threading.enumerate()):
    print(index, thread.name, thread.is_alive())
    if "M12" in thread.name:
        lock_ = True
print(lock_)
print("I took " + str(time.time() - t))
time.sleep(time_sleep / 2)
