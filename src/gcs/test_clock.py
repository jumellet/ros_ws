import time

period = 16 # in msèytds
desired_interval = period
events = 0
time_now = time.time_ns()
time_end = time_now + 1
while time_now < time_end:
    events += 1
    print(events)
    time.sleep(desired_interval)
    time_now = time.time_ns()