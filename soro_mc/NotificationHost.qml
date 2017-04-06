/*
 * Copyright 2017 The University of Oklahoma.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


import QtQuick 2.7
import QtGraphicalEffects 1.0

Item {
    id: host
    width: 900
    height: 1200

    /*function show(title, text) {
        if(messagesBeingDisplayed == 0){
            messages.push(eval("message" + messagesBeingDisplayed))
            //show message at the top of the host pane, centered
            messages[messagesBeingDisplayed].anchors.horizontalCenter: parent.horizontalCenter
            messages[messagesBeingDisplayed].notificationTitleLabel.text = title
            messages[messagesBeingDisplayed].notificationLabel.text = text
            messages[messagesBeingDisplayed].notificationPane.state = "visible"
        } else if (messagesBeingDisplayed < 5) {
        //anchor new message to bottom of the message below it
        messages[messagesBeingDisplayed].anchors.horizontalCenter: messages[messagesBeingDisplayed - 1].horizontalCenter
        messages[messagesBeingDisplayed].notificationTitleLabel.text = title
        messages[messagesBeingDisplayed].notificationLabel.text = text
        messages[messagesBeingDisplayed].notificationPane.state = "visible"
        } else {

           }

        // increase messages in queue
        messagesBeingDisplayed++

        // after some time shift array to remove message
        setTimeout(function(){

             messages[0].anchors.horizontalCenter: parent.horizontalCenter
             messages[0].notificationTitleLabel.text = title
             messages[0].notificationLabel.text = text
             messages[0].notificationPane.state = "visible"
             messages.shift()
             messagesBeingDisplayed--
            })
        }, 100);
    }*/


    /*Message {
        id: message0
        anchors.top: parent.top
    }
    Message {
        id: message1
        anchors.top: message0.visible ? message0.bottom : parent.top
    }
    Message {
        id: message2
        anchors.top: message1.visible ? message1.bottom : parent.top
    }
    Message {
        id: message3
        anchors.top: message2.visible ? message2.bottom : parent.top
    }
    Message {
        id: message4
        anchors.top: message3.visible ? message3.bottom : parent.top
    }*/
}
