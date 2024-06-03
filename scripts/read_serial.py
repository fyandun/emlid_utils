#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

# Replace 'COM3' with the appropriate port name for your system
serial_port = '/dev/ttyACM0'
baud_rate = 38400

# Establish a connection to the serial port
ser = serial.Serial(serial_port, baud_rate, timeout=1)
def publish_odom():    
    pub = rospy.Publisher('/gps/rtk_fix', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        serial_data = ser.readline().decode('utf-8').strip().split()
        lat_x = float(serial_data[2])
        lon_y = float(serial_data[3])
        alt_z = float(serial_data[4])

        #this applies for LLH message type
        sdn = float(serial_data[7]) #standard deviation north
        sde = float(serial_data[8]) #standard deviation east
        sdu = float(serial_data[9]) #standard deviation up

        if serial_data:
            rtk_msg = PoseWithCovarianceStamped()
            rtk_msg.header.frame_id = '/gps'
            rtk_msg.header.stamp = rospy.Time.now()
            rtk_msg.pose.pose.position.x = lat_x
            rtk_msg.pose.pose.position.y = lon_y
            rtk_msg.pose.pose.position.z = alt_z

            rtk_msg.pose.covariance = np.array([pow(sde,2),   0,   0,   0,   0,   0,
                                                0,   pow(sdn,2),   0,   0,   0,   0,
                                                0,   0,   0,   pow(sdu,2),   0,   0,
                                                0,   0,   0,   0,   0,   0,
                                                0,   0,   0,   0,   0,   0,
                                                0,   0,   0,   0,   0,   0])

            print(serial_data)
            pub.publish(rtk_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        publish_odom()
    except rospy.ROSInterruptException:
        print("Closing the serial port.")
        ser.close()
        pass


# try:
#     while True:
#         # Read data from the serial port
#         data = ser.readline().decode('utf-8').strip()

#         # If data received, print it
#         if data:
#             print("Received data from serial port: ", data)
#             # Give the device time to send data again
#             time.sleep(0.5)

# # To close the serial port gracefully, use Ctrl+C to break the loop
# except KeyboardInterrupt:
#     print("Closing the serial port.")
#     ser.close()    