#!/usr/bin/env python3

import psutil
import shutil


def check_disk_usage(disk):
    du  = shutil.disk_usage(disk)
    free = du.free / du.total * 100
    return free 

def check_cpu_usage():
    usage = psutil.cpu_percent(1)
    return usage 

if not (check_disk_usage("/") > 20) or not (check_cpu_usage() < 75):
    print("[ERROR] Disk usage:{check_disk_usage("/")}, Cpu usage: {check_cpu_usage()}")
