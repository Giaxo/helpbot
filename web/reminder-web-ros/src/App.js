import React, { useState, useEffect } from 'react'

import ROSLIB from 'roslib'

import './App.css';

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

  const [event, setEvent] = useState(null)
  const [startTime, setStartTime] = useState(null)
  const [endTime, setEndTime] = useState(null)


  var getReminderClient = new ROSLIB.Service({
    ros : ros,
    name : '/get_reminder',
    serviceType : 'helpbot/GetReminder'
  })

  function getReminder() {
    let getReminderRequest = new ROSLIB.ServiceRequest()
    getReminderClient.callService(getReminderRequest, function(result) {
      let remindEvent = JSON.parse(result.event)
      setStartTime(remindEvent.startTime[0]+":"+remindEvent.startTime[1])
      setEndTime(remindEvent.endTime[0]+":"+remindEvent.endTime[1])
      setEvent(remindEvent)
    })
  }

  function handlerQuit() {
    window.open("about:blank", "_self");
    window.close();
  }

  //Update events when page loads
  useEffect(()=>{
    getReminder()
  }, [])

  return (
    <div className="App">
      <div className="event-container">
        <div className="event-title">
          Promemoria
        </div>
        <div className="event-label">
          Ora:
          <div className="event-time">
            {event!==null?startTime:''} - {event!==null?endTime:''}
          </div>
        </div>
        <div className="event-label">
          <div className="event-name">
            {event!==null?event.name:''}
          </div>
        </div>
        <div className="event-close" onClick={handlerQuit}>
          Chiudi
        </div>
      </div>
    </div>
  );
}

export default App;
