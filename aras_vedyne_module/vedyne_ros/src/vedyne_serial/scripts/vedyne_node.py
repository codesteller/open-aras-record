#!/usr/bin/env python3

'''
 # @ Author: Pallab Maji
 # @ Create Time: 2024-02-06 10:53:24
 # @ Modified time: 2024-02-06 10:53:28
 # @ Description: Enter description here
 '''



import rospy
from rosserial_arduino import SerialClient
from serial import SerialException
from time import sleep

import sys

if __name__=="__main__":

    rospy.init_node("vedyne_node", anonymous=True, log_level=rospy.DEBUG)  # log_level=rospy.DEBUG
    rospy.loginfo("ROS Serial Python Node")

    port_name = rospy.get_param('~port','/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud','57600'))

    # Number of seconds of sync failure after which Arduino is auto-reset.
    # 0 = no timeout, auto-reset disabled
    auto_reset_timeout = int(rospy.get_param('~auto_reset_timeout','0'))

    # for systems where pyserial yields errors in the fcntl.ioctl(self.fd, TIOCMBIS, \
    # TIOCM_DTR_str) line, which causes an IOError, when using simulated port
    fix_pyserial_for_test = rospy.get_param('~fix_pyserial_for_test', False)

    # TODO: do we really want command line params in addition to parameter server params?
    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 2 :
        port_name  = sys.argv[1]

    while not rospy.is_shutdown():
        rospy.loginfo("Connecting to %s at %d baud" % (port_name, baud))
        try:
            client = SerialClient(port_name, baud, fix_pyserial_for_test=fix_pyserial_for_test, auto_reset_timeout=auto_reset_timeout)
            client.run()
        except KeyboardInterrupt:
            break
        except SerialException:
            sleep(1.0)
            continue
        except OSError:
            sleep(1.0)
            continue