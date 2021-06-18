import React, { useState } from 'react'

import DatePicker from 'react-datepicker'
import "react-datepicker/dist/react-datepicker.css";
import { registerLocale, setDefaultLocale } from  "react-datepicker";
import it from 'date-fns/locale/it'
import setHours from "date-fns/setHours";
import setMinutes from "date-fns/setMinutes";

import './NewEventOverlay.css'

registerLocale('it', it)
setDefaultLocale('it');

function NewEventOverlay(props) {

  const [newEventName, setNewEventName] = useState("");
  const [newEventIsWeekly, setNewEventIsWeekly] = useState(false);
  const [newEventDate, setNewEventDate] = useState(new Date());
  const [newEventWeekday, setNewEventWeekday] = useState("0");
  const [newEventStartTime, setNewEventStartTime] = useState(new Date());
  const [newEventEndTime, setNewEventEndTime] = useState(new Date());
  const [newEventType, setNewEventType] = useState("Promemoria");


  return (
    <div className="new-event-overlay" style={props.show?{}:{display: 'none'}}>
        <div className="new-event-overlay-content">
          <img className="new-event-overlay-close" src="plus.png" onClick={props.onClose} alt="close overlay plus" />
          
          <form className="new-event-overlay-form" onSubmit={(e)=>props.onSubmit(e, newEventName, newEventIsWeekly, newEventDate, newEventWeekday, newEventStartTime, newEventEndTime, newEventType)}>
            <div className="new-event-overlay-input">
              <div className="new-event-overlay-label">
                Nome evento:
              </div>
              <input className="new-event-overlay-text" type="text" value={newEventName} onChange={(event) => {setNewEventName(event.target.value)}} />
            </div>
            <div className="new-event-overlay-input">
              <label className="new-event-overlay-label">
              <input type="checkbox" id="isWeekly" name="newEventIsWeekly" checked={newEventIsWeekly} onChange={(event)=>{setNewEventIsWeekly(event.target.checked)}}/>
                Settimanale
              </label>
            </div>
            <div className="new-event-overlay-input">
              <div className="new-event-overlay-label">
                Seleziona il giorno:
              </div>
              <div style={newEventIsWeekly?{display: 'none'}:{}}>
                <DatePicker
                  className="new-event-overlay-datepicker"
                  selected={newEventDate}
                  onChange={(date) => setNewEventDate(date)}
                  dateFormat="dd/MM/yyyy"
                  locale="it"
                  onChangeRaw={(e)=>{e.preventDefault()}}
                  strictParsing
                />
              </div>
              <select className="new-event-overlay-text" name="newEventWeekday" id="newEventWeekday" value={newEventWeekday} onChange={(event)=>{setNewEventWeekday(event.target.value)}} style={newEventIsWeekly?{}:{display: 'none'}}>
                <option value="0">Lunedì</option>
                <option value="1">Martedì</option>
                <option value="2">Mercoledì</option>
                <option value="3">Giovedì</option>
                <option value="4">Venerdì</option>
                <option value="5">Sabato</option>
                <option value="6">Domenica</option>
              </select>
            </div>
            <div className="new-event-overlay-input">
              <div className="new-event-overlay-label">
                Seleziona l'ora iniziale:
              </div>
              <DatePicker
                className="new-event-overlay-datepicker"
                selected={newEventStartTime}
                onChange={(date) => setNewEventStartTime(date)}
                locale="it"
                showTimeSelect
                showTimeSelectOnly
                timeIntervals={5}
                timeCaption="Time"
                dateFormat="HH:mm"
                onChangeRaw={(e)=>{e.preventDefault()}}
                strictParsing
              />
            </div>
            <div className="new-event-overlay-input">
              <div className="new-event-overlay-label">
                Seleziona l'ora finale:
              </div>
              <DatePicker
                className="new-event-overlay-datepicker"
                selected={newEventEndTime}
                onChange={(date) => setNewEventEndTime(date)}
                locale="it"
                showTimeSelect
                showTimeSelectOnly
                timeIntervals={5}
                minTime={newEventStartTime.getTime()}
                maxTime={setHours(setMinutes(new Date(), 59), 23)}
                timeCaption="Time"
                dateFormat="HH:mm"
                onChangeRaw={(e)=>{e.preventDefault()}}
                strictParsing
              />
            </div>
            <div className="new-event-overlay-input">
              <div className="new-event-overlay-label">
                Seleziona il tipo:
              </div>
              <select className="new-event-overlay-text" name="newEventType" id="newEventType" value={newEventType} onChange={(event)=>{setNewEventType(event.target.value)}}>
                {props.eventTypes.map(x => <option key={x} value={x}>{x}</option>)}
              </select>
            </div>
            <input className="new-event-overlay-submit" type="submit" value="Crea evento" />
          </form>

        </div>
      </div>
  );
}

export default NewEventOverlay;