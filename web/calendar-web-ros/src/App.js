import React, { useState, useEffect } from 'react'

import { Calendar, momentLocalizer } from 'react-big-calendar'
import "react-big-calendar/lib/css/react-big-calendar.css";
import moment from 'moment'
import 'moment/locale/it';

import { RRule } from 'rrule'

import ROSLIB from 'roslib'

import './App.css';
import NewEventOverlay from './NewEventOverlay'
import DetailsOverlay from './DetailsOverlay'

const localizer = momentLocalizer(moment)

var ros = new ROSLIB.Ros({
  url : 'ws://192.168.1.66:9090'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});

function App() {

  const [myEvents, setMyEvents] = useState([])
  const [eventTypes, setEventTypes] = useState([])
  const [currCalendarDate, setCurrCalendarDate] = useState(new Date())
  const [showNewEventOverlay, setShowNewEventOverlay] = useState(false)
  const [showDetailsOverlay, setShowDetailsOverlay] = useState(false)
  const [selectedEvent, setSelectedEvent] = useState(null)
  
  var readCalendarClient = new ROSLIB.Service({
    ros : ros,
    name : '/read_calendar',
    serviceType : 'helpbot/ReadCalendar'
  })

  var readTypesClient = new ROSLIB.Service({
    ros : ros,
    name : '/read_types',
    serviceType : 'helpbot/ReadTypes'
  })

  var addEventClient = new ROSLIB.Service({
    ros : ros,
    name : '/add_event',
    serviceType : 'helpbot/AddEvent'
  })

  var removeEventClient = new ROSLIB.Service({
    ros : ros,
    name : '/remove_event',
    serviceType : 'helpbot/RemoveEvent'
  })

  const weekDaysList = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]

  const messages = {
    allDay: 'Giorno',
    previous: 'Precedente',
    next: 'Successivo',
    today: 'Oggi',
    month: 'Mese',
    week: 'Settimana',
    day: 'Giorno',
    agenda: 'Agenda',
    date: 'Data',
    time: 'Ora',
    event: 'Evento',
    showMore: total => `+ ${total} altri eventi`
  }

  //Reads events from ROS calendar
  function updateEvents(date, view) {

    setCurrCalendarDate(date)

    myEvents.length=0

    let currYear = moment(date).format("YYYY")
    let currMonth = moment(date).format("MM")

    let readRequest = new ROSLIB.ServiceRequest({
      year : currYear,
      month : currMonth
    })

    readCalendarClient.callService(readRequest, function(result) {

      let completeCalendar = JSON.parse(result.calendar)
      let dailyCalendar = completeCalendar.daily
      let weeklyCalendar = completeCalendar.weekly
      //console.log(dailyCalendar)

      let newEventsArray = []
      //Load events from daily calendar
      for (let keyDay in dailyCalendar) {
        let currDay = dailyCalendar[keyDay]
        //console.log(currDay)
        for (let keyEvent in currDay) {
          let currEvent = currDay[keyEvent]
          let newEvent = {}
          newEvent["start"] = moment(currYear+"-"+currMonth+"-"+keyDay+" "+currEvent["startTime"][0]+":"+currEvent["startTime"][1]).toDate()
          newEvent["end"] = moment(currYear+"-"+currMonth+"-"+keyDay+" "+currEvent["endTime"][0]+":"+currEvent["endTime"][1]).toDate()
          newEvent["title"] = currEvent["name"]
          newEvent["resource"] = {"type":currEvent["type"],"isWeekly":false}
          newEventsArray.push(newEvent)
        }
      }

      //Load events from weekly calendar
      let dayNum = 0
      for (let keyDay in weeklyCalendar) {
        let currDay = weeklyCalendar[keyDay]
        
        let rule = new RRule({
          freq: RRule.WEEKLY,
          interval: 1,
          byweekday: dayNum,
          dtstart: new Date(Date.UTC(parseInt(currYear), parseInt(currMonth)-1, 1)),
          until: new Date(Date.UTC(parseInt(currYear), parseInt(currMonth), 0))
        })
        let allCurrWeekdays = rule.all()

        for (let keyEvent in currDay) {
          for (let nextDate of allCurrWeekdays) {
            //console.log(nextDate)
            let currEvent = currDay[keyEvent]
            let newEvent = {}
            newEvent["start"] = new Date(nextDate.setHours(currEvent["startTime"][0], currEvent["startTime"][1]))
            newEvent["end"] = new Date(nextDate.setHours(currEvent["endTime"][0], currEvent["endTime"][1]))
            newEvent["title"] = currEvent["name"]
            newEvent["resource"] = {"type":currEvent["type"],"isWeekly":true}
            newEventsArray.push(newEvent)
          }
        }
        dayNum+=1
      }
      setMyEvents(newEventsArray);

      //Read event types
      let readTypesRequest = new ROSLIB.ServiceRequest()
      readTypesClient.callService(readTypesRequest, function(result) {
        setEventTypes(JSON.parse(result.types))
      })

    }, function(error) {
      console.log('NO SERVICE CONNECTION')
    })
  }

  //Add new event to ROS calendar
  function handlerNewEventSubmit(event, newEventName, newEventIsWeekly, newEventDate, newEventWeekday, newEventStartTime, newEventEndTime, newEventType) {

    event.preventDefault()
    
    if (newEventName!=="" && newEventDate!==null && newEventStartTime!==null && newEventEndTime!==null) {

      setShowNewEventOverlay(false)

      let newEvent = {
        name : newEventName,
        startTime : [moment(newEventStartTime).format("HH"), moment(newEventStartTime).format("mm")],
        endTime : [moment(newEventEndTime).format("HH"), moment(newEventEndTime).format("mm")],
        type : newEventType
      }
      let newDate
      if (newEventIsWeekly) {
        newDate = weekDaysList[newEventWeekday]
      } else {
        newDate = {
          year : moment(newEventDate).format("YYYY"),
          month : moment(newEventDate).format("MM"),
          day : moment(newEventDate).format("DD")
        }
      }
      let addRequest = new ROSLIB.ServiceRequest({
        "event" : JSON.stringify(newEvent),
        "date" : JSON.stringify(newDate),
        "isWeekly" : newEventIsWeekly
      })

      addEventClient.callService(addRequest, function(result) {
        if(result.resultCode==="0") {
          updateEvents(currCalendarDate)
        } else {
          console.log("ERROR IN ADD EVENT SERVICE CALL")
        }
      })

    }
  }

  //Toggle visibility of the new event overlay
  function newEventOverlayToggle() {
    setShowNewEventOverlay(!showNewEventOverlay)
  }

  //Handler for the onSelect event in the calendar
  function handlerOnSelectEvent(selEvent, e) {
    setSelectedEvent(selEvent)
    detailsOverlayToggle()
  }

  //Toggle visibility of the details overlay
  function detailsOverlayToggle() {
    setShowDetailsOverlay(!showDetailsOverlay)
  }

  //Remove event from ros calendar
  function handlerRemoveEvent(event) {

    setShowDetailsOverlay(false)

    let name = event.title
    let isWeekly = event.resource.isWeekly

    let date
    if (isWeekly) {
      date = weekDaysList[(event.start.getDay()+6)%7]
    } else {
      date = {
        year : moment(event.start).format("YYYY"),
        month : moment(event.start).format("MM"),
        day : moment(event.start).format("DD")
      }
    }

    let removeRequest = new ROSLIB.ServiceRequest({
      "eventName" : name,
      "date" : JSON.stringify(date),
      "isWeekly" : isWeekly
    })
    removeEventClient.callService(removeRequest, function(result) {
      if(result.resultCode==="0") {
        updateEvents(currCalendarDate)
      } else {
        console.log("ERROR IN REMOVE EVENT SERVICE CALL")
      }
    })
  }

  //Update events when page loads
  useEffect(()=>{
    updateEvents(moment().toDate())
  }, [])

  //DEBUG
  //useEffect(()=>{
  //  console.log("EVENTI: "+myEvents)
  //})

  return (

    <div className="App">
      <div className="calendar-header">
        <div className="new-event-button" onClick={newEventOverlayToggle}>
          <img className="new-event-plus" src="plus.png" alt="add avent plus" />
          Nuovo evento
        </div>
      </div>
      <div className="calendar-container">
        <Calendar
          localizer={localizer}
          events={myEvents}
          views={['month', 'day', 'agenda']}
          messages={messages}
          onNavigate={updateEvents}
          onSelectEvent={handlerOnSelectEvent}
        />
      </div>
      <NewEventOverlay
        show={showNewEventOverlay}
        onSubmit={handlerNewEventSubmit}
        onClose={newEventOverlayToggle}
        eventTypes={eventTypes}
      />
      <DetailsOverlay 
        show={showDetailsOverlay}
        onRemoveClick={handlerRemoveEvent}
        onClose={detailsOverlayToggle}
        selEvent={selectedEvent}
      />
    </div>

  );
}

export default App;

//TODO
//Implement remove event
//Get ros ip from external file or setup proxy