/*
 * Copyright 2017 Jacob Jordan <doublejinitials@ou.edu>
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

import QtQuick 2.0

Item {
    width: 200
    height: 600
    id: viewSelector

    property int selectedViewIndex: -1
    readonly property ListModel views: viewListModel
    property real aspectRatio: 1.77
    readonly property int viewCount: viewListModel.count

    property int spacing: 10

    readonly property string selectedViewTitle: list.selectedViewTitle
    readonly property bool selectedViewIsStreaming: list.selectedViewIsStreaming
    readonly property string selectedViewStreamProfile: list.selectedViewStreamProfile

    signal viewClicked(int index)

    function addItem(shaderSource, title) {
        viewListModel.append({"item_title": title, "item_index": viewCount, "item_source": shaderSource, "item_streaming": false, "item_profile": "", "item_selected": false})
        if (viewListModel.count == 1) {
            // This is the first item added
            selectedViewIndex = 0
        }
    }

    function removeItem(index) {
        viewListModel.remove(index)
    }

    function clearItems() {
        viewListModel.clear()
    }

    onSelectedViewIndexChanged: {
        for (var i = 0; i < viewListModel.count; ++i) {
            viewListModel.get(i).item_selected = i == selectedViewIndex
        }
    }

    function setViewTitle(viewIndex, title) {
        if (viewIndex < viewListModel.count) {
            viewListModel.get(viewIndex).item_title = title
        }
    }

    function setViewShaderSource(viewIndex, source) {
        if (viewIndex < viewListModel.count) {
            viewListModel.get(viewIndex).item_source = source
        }
    }

    function setViewShaderSourceRect(viewIndex, rect) {
        if (viewIndex < viewListModel.count) {
            viewListModel.get(viewIndex).item_source_rect = rect
        }
    }

    function setViewIsStreaming(viewIndex, streaming) {
        if (viewIndex < viewListModel.count) {
            viewListModel.get(viewIndex).item_streaming = streaming
        }
    }

    function setViewStreamProfileName(viewIndex, name) {
        if (viewIndex < viewListModel.count) {
            viewListModel.get(viewIndex).item_profile = name
        }
    }

    ListView
    {
        id: list
        anchors.top: parent.top
        anchors.topMargin: viewSelector.spacing
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        clip: true

        property string selectedViewTitle
        property bool selectedViewIsStreaming
        property string selectedViewStreamProfile

        model: ListModel
        {
            id: viewListModel
        }

        delegate: Item {
            id: listItem
            height: thumbnail.height + viewSelector.spacing
            anchors.left: parent.left
            anchors.right: parent.right
            ItemThumbnailView {
                x: 0
                y: 0
                width: parent.width
                height: width / viewSelector.aspectRatio
                id: thumbnail
                text: item_title
                streamProfile: item_profile
                streaming: item_streaming
                selected: item_selected
                shaderSource: item_source

                onSelectedChanged: {
                    if (selected) {
                        list.contentY = listItem.y - listItem.anchors.topMargin
                        list.selectedViewTitle = text
                        list.selectedViewStreamProfile = streamProfile
                        list.selectedViewIsStreaming = streaming
                    }
                }

                onStreamProfileChanged: {
                    if (selected) {
                        list.selectedViewStreamProfile = streamProfile
                    }
                }

                onStreamingChanged: {
                    if (selected) {
                        list.selectedViewIsStreaming = streaming
                    }
                }

                onClicked: {
                    viewClicked(item_index)
                }
            }
        }
    }
}
