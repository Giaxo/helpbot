#!/usr/bin/env python
import rospy
from datetime import datetime

updateDelay = 5

def updateTimestamp(e):
    #Get current timestamp
    now = datetime.now()
    timestamp = now.strftime("%d/%m/%Y %H:%M:%S")
    day = timestamp[0:2]
    month = timestamp[3:5]
    year = timestamp[6:10]
    hour = timestamp[11:13]
    minute = timestamp[14:16]
    #Save timestamp on parameter server
    rospy.set_param('timestamp', {'day': day, 'month': month, 'year': year, 'hour': hour, 'minute': minute})

def main():
    #Start node
    rospy.init_node("timeserver")
    #Start timer to update timestamp (first time manually)
    updateTimestamp(None)
    rospy.Timer(rospy.Duration(updateDelay), updateTimestamp)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass