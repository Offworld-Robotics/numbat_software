#!/usr/bin/env python3

import os
import fcntl
import sys

device = sys.argv[1]
print("resetting driver:" + device)
USBDEVFS_RESET= 21780

try:
    with open(device ,'w', os.O_WRONLY) as f:
        fcntl.ioctl(f, USBDEVFS_RESET, 0)
        print("Successfully reset usb")
except Exception as msg:
    print("Failed to reset device: " + msg)
