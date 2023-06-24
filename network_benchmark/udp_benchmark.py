"""

Robot controller
----------------

https://github.com/turingbirds/robot-driver

Copyright 2023
Apache License 2.0

"""

import datetime
import numpy as np
import json
import os
import time
import matplotlib
import matplotlib.pyplot as plt


def make_plot_of_ping_times(ping_times, fname_snip="", title_snip_pre="", title_snip_post=""):
    bins = np.arange(np.amin(ping_times), np.amax(ping_times), step=5)
    hist, bin_edge = np.histogram(ping_times, bins=bins, density=True)

    fig, ax = plt.subplots()
    ax.bar(bin_edge[:-1], hist, width=np.diff(bin_edge), align="edge")
    ax.set_xlim(0, np.amax(ping_times) + 6)
    ax.set_yscale("log")
    fig.suptitle(title_snip_pre + "Data points: " + str(len(ping_times)) + ", Max ping: " + str(np.amax(ping_times)) + " ms" + title_snip_post)
    fig.savefig("c:\Temp\ping_times_" + fname_snip +  ".png")


import socket

UDP_IP = "192.168.1.229"
UDP_PORT = 1234
MESSAGE = "test from Python " + str(datetime.datetime.now())

udp_timeout_cutoff = .1    # timeout in seconds

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(udp_timeout_cutoff) # timeout in seconds

def make_request(MESSAGE, UDP_IP, UDP_PORT):
    sock.sendto(bytes(MESSAGE, "utf-8"), (UDP_IP, UDP_PORT))

    try:
        data, addr = sock.recvfrom(1024)   # receive up to 1024 bytes of data
        print("received data" + str(data))
    except socket.error:
        pass

    


def take_samples(N_samples=1000):
    samples = []

    while True:
        before = datetime.datetime.now()
        make_request(MESSAGE, UDP_IP, UDP_PORT)
        after = datetime.datetime.now()

        print("Time elapsed: " + str(after - before))

        samples.append(str(after - before))
        
        if len(samples) > N_samples:
            break

    return samples

samples = take_samples()
timeout_cutoff_ms = 999 * udp_timeout_cutoff




n_timeouts = 0
ping_times= []
for s in samples:
    try:
        dt = datetime.datetime.strptime(s, r"%H:%M:%S.%f")
    except:
        try:
            dt = datetime.datetime.strptime(s, r"%H:%M:%S")
        except:
            n_timeouts += 1
            continue
           
    ping_time = int(float(dt.strftime("%f")) / 1000.)
    print("--> " + str(ping_time))
    if ping_time > timeout_cutoff_ms:
        n_timeouts += 1
        continue
    ping_times.append(ping_time)
    
make_plot_of_ping_times(ping_times, fname_snip="udp",  title_snip_pre="UDP request times, ", title_snip_post=" (" + str(n_timeouts) + " timeouts)")
