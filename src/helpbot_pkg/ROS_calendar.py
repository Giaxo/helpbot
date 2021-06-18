from __future__ import division
from posixpath import expanduser
import jsonpickle
import calendar
import datetime
import os, errno

class Calendar(object):
    def __init__(self, filepath=expanduser("~")+"/ROS_calendar.save"):
        self.filepath = filepath
        self.weekCalendar = {"Monday": [], "Tuesday": [], "Wednesday": [], "Thursday": [], "Friday": [], "Saturday": [], "Sunday": []}
        self.calendar = {}
        self.loadCalendar()
    
    def addEventWeek(self, event, weekday):
        if not (self.findEventWeek(event.name, weekday) == None):
            #TODO ROS ERROR OR SOME KIND OF ERROR
            print("NOME EVENTO ESISTENTE")
            return False
        self.weekCalendar[weekday].append(event)
        self.saveCalendar()
        return True
    
    def addEventDay(self, event, date):
        if date.year not in self.calendar:
            self.calendar.update({date.year: {}})
        if date.month not in self.calendar[date.year]:
            self.calendar[date.year].update({date.month: {}})
        if date.day not in self.calendar[date.year][date.month]:
            self.calendar[date.year][date.month].update({date.day: []})
            self.calendar[date.year][date.month][date.day].append(event)
        else:
            if not (self.findEventDay(event.name, date) == None):
                #TODO ROS ERROR OR SOME KIND OF ERROR
                print("NOME EVENTO ESISTENTE")
                return False
            self.calendar[date.year][date.month][date.day].append(event)
        self.saveCalendar()
        return True

    def findEventWeek(self, name, weekday):
        for event in self.weekCalendar[weekday]:
            if event.name == name:
                return event
        return None

    def findEventDay(self, name, date):
        for event in self.calendar[date.year][date.month][date.day]:
            if event.name == name:
                return event
        return None

    def removeEventWeek(self, name, weekday):
        print("REMOVING WEEKLY "+name+";"+weekday)
        event = self.findEventWeek(name, weekday)
        if (event==None):
            print("EVENTO DA RIMUOVERE NON TROVATO")
            return False
        self.weekCalendar[weekday].remove(event)
        self.saveCalendar()
        return True

    def removeEventDay(self, name, date):
        print("REMOVING "+name)
        event = self.findEventDay(name, date)
        if (event==None):
            print("EVENTO DA RIMUOVERE NON TROVATO")
            return False
        self.calendar[date.year][date.month][date.day].remove(event)
        self.saveCalendar()
        return True

    def readCalendarMonth(self, year, month):
        completeCalendar = {
            "daily": None,
            "weekly": self.weekCalendar
        }
        if year in self.calendar:
            if month in self.calendar[year]:
                completeCalendar["daily"] = self.calendar[year][month]
        return jsonpickle.encode(completeCalendar)
    
    def saveCalendar(self):
        try:
            directory = os.path.dirname(self.filepath)
            try:
                os.makedirs(directory)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    print("Error when creating the save file path: " + str(e))
                    raise
            completeCalendar = {
                "daily": self.calendar,
                "weekly": self.weekCalendar
            }
            calendarJson = jsonpickle.encode(completeCalendar)
            with open(self.filepath, 'w') as f:
                f.write(calendarJson)
        except Exception as e:
            print("SaveCalendar error: " + str(e))
            raise

    def loadCalendar(self):
        try:
            with open(self.filepath, 'r') as f:
                calendarJson = f.read()
            completeCalendar = jsonpickle.decode(calendarJson, classes=[Event, Date])
            self.calendar = completeCalendar["daily"]
            self.weekCalendar = completeCalendar["weekly"]
        except Exception as e:
            print("Save file not existing, creating new one")

    def checkEventsNow(self, timestamp):
        foundEvents = []
        year = timestamp["year"]
        month = timestamp["month"]
        day = timestamp["day"]
        hour = timestamp["hour"]
        minute = timestamp["minute"]
        currHdM = int(hour)+int(minute)/100
        weekday = calendar.day_name[datetime.datetime(int(year), int(month), int(day)).weekday()]
        #Check in weekly calendar if there are events to activate
        for event in self.weekCalendar[weekday]:
            eventStartHdM = event.getStartHdM()
            eventEndHdM = event.getEndHdM()
            if currHdM >= eventStartHdM and currHdM <= eventEndHdM and event.finished==False:
                foundEvents.append(event)
                event.finished = True
        #Check in daily calendar if there are events to activate
        if year in self.calendar:
            if month in self.calendar[year]:
                if day in self.calendar[year][month]:
                    for event in self.calendar[year][month][day]:
                        eventStartHdM = event.getStartHdM()
                        eventEndHdM = event.getEndHdM()
                        if currHdM >= eventStartHdM and currHdM <= eventEndHdM and event.finished==False:
                            foundEvents.append(event)
                            event.finished = True
        self.saveCalendar()
        return foundEvents


class Event(object):
    def __init__(self, name, startTime, endTime, type):
        self.name = name
        self.startTime = startTime
        self.endTime = endTime
        self.type = type
        self.finished = False
    def getStartHdM(self):
        return int(self.startTime[0])+int(self.startTime[1])/100
    def getEndHdM(self):
        return int(self.endTime[0])+int(self.endTime[1])/100

class Date(object):
    def __init__(self, year, month, day):
        self.year = year
        self.month = month
        self.day = day

#DONE
#Create 2 structures for calendar:
#   one for weekly scheduling
#   one for day to day scheduling
#The weekly structure is just a list/dictionary with every day of the week containing a list of events
#The daily structure is a dictionary of dictionaries like a tree: year -> month -> day
#Every day contains a list of events
#The dictionaries get filled only when a task is added in that year/month/day and the entry is removed when empty
