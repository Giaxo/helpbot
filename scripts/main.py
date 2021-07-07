#!/usr/bin/env python
import rospy
import ast
import sys
import jsonpickle
import os, rospkg
import subprocess

import roslaunch
from std_msgs.msg import String
from helpbot.srv import AddEvent, RemoveEvent, ReadCalendar, ReadTypes, GetReminder

from helpbot_pkg.ROS_calendar import *
from helpbot_pkg.eventTypes import eventTypes

def activateEvents(e):
    #print("Checking events")
    currts = rospy.get_param('timestamp')
    #print(currts)
    foundEvents = myCalendar.checkEventsNow(currts)
    if foundEvents:
        print("Activating events:")
        for event in foundEvents:
            print(event.name+": "+event.type)
            activeEventsPub.publish(jsonpickle.encode(event))
            if(event.type=="Promemoria"):
                startReminder(event)
            else:
                roslaunch.pmon._init_signal_handlers = lambda: None
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                #print("UUID: "+uuid)
                roslaunch.configure_logging(uuid)
                launch = roslaunch.parent.ROSLaunchParent(uuid, [eventTypes[event.type]])
                launch.start()
                launchedEventsList.append(launch)


def startReminder(event):
    eventsToRemind.append(event)
    rp = rospkg.RosPack()
    script_dir = os.path.join(rp.get_path("helpbot"), "web")
    script_path = os.path.join(script_dir, "startReminder.sh")
    print(script_path)
    subprocess.Popen([script_path, script_dir])

def handlerGetReminder(req):
    if len(eventsToRemind)>0:
        return jsonpickle.encode(eventsToRemind.pop(0))
    return jsonpickle.encode([])

#Handler for the AddEvent service (req: event, date, isWeekly; res: resultCode)
def handlerAddEvent(req):
    print("ADD EVENT\n"+str(req))
    eventDict = ast.literal_eval(req.event)
    eventToAdd = Event(eventDict["name"], eventDict["startTime"], eventDict["endTime"], eventDict["type"])
    if req.isWeekly == True:
        res = myCalendar.addEventWeek(eventToAdd, ast.literal_eval(req.date))
    else:
        dateDict = ast.literal_eval(req.date)
        dateToAdd = Date(dateDict["year"], dateDict["month"], dateDict["day"])
        res = myCalendar.addEventDay(eventToAdd, dateToAdd)
    if res:
        return "0"
    else:
        return "1"

#Handler for the RemoveEvent service (req: event, date, isWeekly; res: resultCode)
def handlerRemoveEvent(req):
    print("REMOVE EVENT\n"+str(req))
    eventName = req.eventName
    eventDate = ast.literal_eval(req.date)
    if req.isWeekly == True:
        res = myCalendar.removeEventWeek(eventName, eventDate)
    else:
        dateToRemove = Date(eventDate["year"], eventDate["month"], eventDate["day"])
        res = myCalendar.removeEventDay(eventName, dateToRemove)
    if res:
        return "0"
    else:
        return "1"

#Handler for the ReadCalendar service (req: year, month; res: calendar)
def handlerReadCalendar(req):
    #print("READ CALENDAR\n"+str(req))
    return myCalendar.readCalendarMonth(req.year, req.month)

#Handler for the ReadTypes service (req: ; res: types)
def handlerReadTypes(req):
    #print("READ TYPES\n"+str(req))
    return jsonpickle.encode(list(eventTypes))

def handlerOnShutdown():
    #Terminate launch files
    for launch in launchedEventsList:
        launch.shutdown()

def main(argv):

    global myCalendar
    global activeEventsPub
    global eventsToRemind
    global launchedEventsList
    global webserverProcess
    global tmp_dir
    launchedEventsList = []
    eventsToRemind = []

    print("Starting ROS_calendar")

    #Start web server
    rp = rospkg.RosPack()
    web_dir = os.path.join(rp.get_path("helpbot"), "web")
    web_script = os.path.join(web_dir, "startCalendar.sh")
    print("Starting web server using: " + web_script)
    subprocess.Popen([web_script, web_dir])

    #Start ros program
    if len(argv) > 1:
        print("Using save file: " + argv[1])
        myCalendar = Calendar(argv[1])
    else:
        myCalendar = Calendar()

    rospy.init_node("ROS_calendar")

    rospy.on_shutdown(handlerOnShutdown)

    activeEventsPub = rospy.Publisher('active_events', String, queue_size=10)

    addEventSrv = rospy.Service('add_event', AddEvent, handlerAddEvent)
    removeEventSrv = rospy.Service('remove_event', RemoveEvent, handlerRemoveEvent)
    readCalendarSrv = rospy.Service('read_calendar', ReadCalendar, handlerReadCalendar)
    readTypesSrv = rospy.Service('read_types', ReadTypes, handlerReadTypes)
    getReminderSrv = rospy.Service('get_reminder', GetReminder, handlerGetReminder)

    rospy.Timer(rospy.Duration(10), activateEvents)
    
    rospy.spin()

if __name__ == "__main__":
    try:
        main(rospy.myargv(argv=sys.argv))
    except rospy.ROSInterruptException:
        pass


#TODO
#Remove dict entry when empty (check when remove event)