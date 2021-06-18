import React from 'react'

import moment from 'moment'
import 'moment/locale/it';

import './DetailsOverlay.css'

function DetailsOverlay(props) {

  var selEvent = props.selEvent

  if (selEvent!=null) {

    var name = selEvent.title
    var isWeekly = selEvent.resource.isWeekly
    var date
    if (isWeekly) {
      date = moment(selEvent.start).format("dddd")
    } else {
      date = moment(selEvent.start).format("DD MMMM YYYY")
    }
    var startTime = moment(selEvent.start).format("HH:mm")
    var endTime = moment(selEvent.end).format("HH:mm")
    var type = selEvent.resource.type

    return (
      <div className="details-overlay" style={props.show?{}:{display: 'none'}}>
          <div className="details-overlay-content">
            <img className="details-overlay-close" src="plus.png" onClick={props.onClose} alt="close overlay plus" />
            
            <div className="details-overlay-details">
              <div className="details-overlay-label">
                Nome evento:
                <div className="details-overlay-info">
                  {name}
                </div>
              </div>
              <div className="details-overlay-label">
                Data:
                <div className="details-overlay-info">
                  {date}
                </div>
              </div>
              <div className="details-overlay-label">
                Ora:
                <div className="details-overlay-info">
                  {startTime} - {endTime}
                </div>
              </div>
              <div className="details-overlay-label">
                Tipo evento:
                <div className="details-overlay-info">
                  {type}
                </div>
              </div>
              <div className="details-overlay-remove-button" onClick={()=>{props.onRemoveClick(selEvent)}}>
                Elimina {isWeekly?'ogni '+date:''}
              </div>
            </div>

          </div>
        </div>
    );

  } else {
    return null;
  }
}

export default DetailsOverlay;