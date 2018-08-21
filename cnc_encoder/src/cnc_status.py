#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
import sys
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int8MultiArray
from cnc_encoder.msg import encoder
from cnc_encoder.msg import limitswitch


def talker():

    num_ls = 1
    num_c = 1

    pub_encoder = rospy.Publisher('encoder', encoder, queue_size=10)
    pub_limit = rospy.Publisher('limit_switch', limitswitch, queue_size=10)

    rospy.init_node('cnc_status', anonymous=True)
    rate = rospy.Rate(2000)  # 10hz

    ls = limitswitch()
    ct = encoder()

    while not rospy.is_shutdown():

        bytesToRead = ser.inWaiting()
        ser.read(0)
        data_raw = ser.readline()
        ls.header.stamp = rospy.get_rostime()
        ls.header.frame_id = "limitswitch"
        ct.header.stamp = rospy.get_rostime()
        ct.header.frame_id = "counter"

        if data_raw.find("LS %d)" % num_ls) is not -1:
            ls.data.append(int(data_raw[7:8]))
            num_ls += 1

        if data_raw.find("C  %d)" % num_c) is not -1:
            ct.data.append(int(data_raw[7:11]))
            num_c += 1

        if num_ls > 6:
            pub_limit.publish(ls)
            # print(limit_switch)
            ls = limitswitch()
            num_ls = 1

        if num_c > 3:
            pub_encoder.publish(ct)
            # print(counter)
            ct = encoder()
            num_c = 1

    rate.sleep()


if __name__ == '__main__':

    try:
        # Tried with and without the last 3 parameters, and also at 1Mbps, same happens.
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=4)
        talker()
    except rospy.ROSInterruptException:
        pass
