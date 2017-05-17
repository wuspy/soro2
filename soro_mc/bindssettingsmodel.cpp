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

#include "bindssettingsmodel.h"
#include "soro_core/constants.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>
#include <QFile>
#include <QDebug>

#define FILE_PATH SORO_MC_SETTINGS_DIR + "/binds.json"

#define KEY_KEY "key"
#define KEY_BUTTON "button"
#define KEY_TOGGLE_FULLSCREEN "toggle_fullscreen"
#define KEY_VIDEO_OFF "video_off"
#define KEY_VIDEO_ON "video_on"
#define KEY_AUDIO_OFF "audio_off"
#define KEY_AUDIO_ON "audio_on"
#define KEY_VIDEO_CAMERA "camera"
#define KEY_AV_PROFILE "profile"

namespace Soro {

void BindsSettingsModel::load() {
    QByteArray rawJson;
    QJsonDocument jsonDocument;
    QJsonParseError jsonError;
    QFile file(FILE_PATH);

    _keyActions.clear();
    _buttonActions.clear();

    if (!file.exists())
    {
        throw QString("The binds settings file \"%1\" does not exist.").arg(FILE_PATH);
    }
    if (!file.open(QIODevice::ReadOnly))
    {
        throw QString("Error opening binds settings file \"%1\".").arg(FILE_PATH);
    }

    rawJson = file.readAll();
    jsonDocument = QJsonDocument::fromJson(rawJson, &jsonError);
    if (jsonError.error != QJsonParseError::NoError)
    {
        throw QString("Error parsing binds settings file \"%1\": %2").arg(FILE_PATH, jsonError.errorString());
    }

    //
    // Parse simple actions (those with no arguments)
    //

    if (jsonDocument.object().contains(KEY_TOGGLE_FULLSCREEN))
    {
        Action action;
        action.type = Action_Toggle_Fullscreen;
        parseAction(jsonDocument.object().value(KEY_TOGGLE_FULLSCREEN), action);
    }

    if (jsonDocument.object().contains(KEY_AUDIO_OFF))
    {
        Action action;
        action.type = Action_Audio_Off;
        parseAction(jsonDocument.object().value(KEY_AUDIO_OFF), action);
    }

    //
    // Parse complex actions (with arguments and possibly more than one binding for different arguments)
    //

    for (QJsonValue value : jsonDocument.object().value(KEY_AUDIO_ON).toArray())
    {
        qDebug() << value;
        Action action;
        action.type = Action_Audio_On;
        action.args = new AudioOnActionArgs();
        int profile = value.toObject().value(KEY_AV_PROFILE).toInt(-1);
        if (profile == -1) {
            throw QString("Error parsing binds settings file \"%1\": Audio on action has an invalid profile index.").arg(FILE_PATH);
        }
        reinterpret_cast<AudioOnActionArgs*>(action.args)->profile = profile;

        parseAction(value, action);
    }

    for (QJsonValue value : jsonDocument.object().value(KEY_VIDEO_OFF).toArray())
    {
        Action action;
        action.type = Action_Video_Off;
        action.args = new VideoOffActionArgs();
        int camera = value.toObject().value(KEY_VIDEO_CAMERA).toInt(-1);
        if (camera == -1) {
            throw QString("Error parsing binds settings file \"%1\": Video Off action has an invalid camera index.").arg(FILE_PATH);
        }
        reinterpret_cast<VideoOnActionArgs*>(action.args)->camera = camera;

        parseAction(value, action);
    }

    for (QJsonValue value : jsonDocument.object().value(KEY_VIDEO_ON).toArray())
    {
        Action action;
        action.type = Action_Video_On;
        action.args = new VideoOnActionArgs();
        int camera = value.toObject().value(KEY_VIDEO_CAMERA).toInt(-1);
        if (camera == -1) {
            throw QString("Error parsing binds settings file \"%1\": Video On action has an invalid camera index.").arg(FILE_PATH);
        }
        reinterpret_cast<VideoOnActionArgs*>(action.args)->camera = camera;

        int profile = value.toObject().value(KEY_AV_PROFILE).toInt(-1);
        if (profile == -1) {
            throw QString("Error parsing binds settings file \"%1\": Video On action has an invalid profile index.").arg(FILE_PATH);
        }
        reinterpret_cast<VideoOnActionArgs*>(action.args)->profile = profile;

        parseAction(value, action);
    }
}

void BindsSettingsModel::parseAction(QJsonValue jsonObject, BindsSettingsModel::Action action)
{
    Qt::Key key = keyStringToKey(jsonObject.toObject().value(KEY_KEY).toString());
    SDL_GameControllerButton button = buttonStringToButton(jsonObject.toObject().value(KEY_BUTTON).toString());
    if (key != Qt::Key_No)
    {
        if (_keyActions.contains(key))
        {
            throw QString("Error parsing binds settings file \"%1\": Key %2 is bound to more than one action.").arg(FILE_PATH, jsonObject.toObject().value(KEY_KEY).toString());
        }
        _keyActions.insert(key, action);
    }
    if (button != SDL_CONTROLLER_BUTTON_INVALID)
    {
        if (_buttonActions.contains(button))
        {
            throw QString("Error parsing binds settings file \"%1\": Gamepad button %2 is bound to more than one action.").arg(FILE_PATH, jsonObject.toObject().value(KEY_BUTTON).toString());
        }
        _buttonActions.insert(button, action);
    }
}

Qt::Key BindsSettingsModel::keyStringToKey(QString key) const
{
    key = key.toLower();
    if (key == "a") return Qt::Key_A;
    if (key == "b") return Qt::Key_B;
    if (key == "c") return Qt::Key_C;
    if (key == "d") return Qt::Key_D;
    if (key == "e") return Qt::Key_E;
    if (key == "f") return Qt::Key_F;
    if (key == "g") return Qt::Key_G;
    if (key == "h") return Qt::Key_H;
    if (key == "i") return Qt::Key_I;
    if (key == "j") return Qt::Key_J;
    if (key == "k") return Qt::Key_K;
    if (key == "l") return Qt::Key_L;
    if (key == "m") return Qt::Key_M;
    if (key == "n") return Qt::Key_N;
    if (key == "o") return Qt::Key_O;
    if (key == "p") return Qt::Key_P;
    if (key == "q") return Qt::Key_Q;
    if (key == "r") return Qt::Key_R;
    if (key == "s") return Qt::Key_S;
    if (key == "t") return Qt::Key_T;
    if (key == "u") return Qt::Key_U;
    if (key == "v") return Qt::Key_V;
    if (key == "w") return Qt::Key_W;
    if (key == "x") return Qt::Key_X;
    if (key == "y") return Qt::Key_Y;
    if (key == "z") return Qt::Key_Z;
    if (key == "0") return Qt::Key_0;
    if (key == "1") return Qt::Key_1;
    if (key == "2") return Qt::Key_2;
    if (key == "3") return Qt::Key_3;
    if (key == "4") return Qt::Key_4;
    if (key == "5") return Qt::Key_5;
    if (key == "6") return Qt::Key_6;
    if (key == "7") return Qt::Key_7;
    if (key == "8") return Qt::Key_8;
    if (key == "9") return Qt::Key_9;
    if (key == "-") return Qt::Key_Minus;
    if (key == "=") return Qt::Key_Equal;
    if (key == "[") return Qt::Key_BraceLeft;
    if (key == "]") return Qt::Key_BraceRight;
    if (key == ";") return Qt::Key_Semicolon;
    if (key == "'") return Qt::Key_QuoteDbl;
    if (key == ",") return Qt::Key_Comma;
    if (key == ".") return Qt::Key_Period;
    if (key == "/") return Qt::Key_Slash;
    if (key == "`") return Qt::Key_QuoteLeft;
    if (key == "f1") return Qt::Key_F1;
    if (key == "f2") return Qt::Key_F2;
    if (key == "f3") return Qt::Key_F3;
    if (key == "f4") return Qt::Key_F4;
    if (key == "f5") return Qt::Key_F5;
    if (key == "f6") return Qt::Key_F6;
    if (key == "f7") return Qt::Key_F7;
    if (key == "f8") return Qt::Key_F8;
    if (key == "f9") return Qt::Key_F9;
    if (key == "f10") return Qt::Key_F10;
    if (key == "f11") return Qt::Key_F11;
    if (key == "f12") return Qt::Key_F12;
    if (key == "tab") return Qt::Key_Tab;
    if (key == "space") return Qt::Key_Space;
    if (key == "\\") return Qt::Key_Backslash;
    if (key == "insert") return Qt::Key_Insert;
    if (key == "delete") return Qt::Key_Delete;
    if (key == "home") return Qt::Key_A;
    if (key == "end") return Qt::Key_A;
    if (key == "page_up") return Qt::Key_A;
    if (key == "page_down") return Qt::Key_A;
    if (key == "up") return Qt::Key_Up;
    if (key == "down") return Qt::Key_Down;
    if (key == "left") return Qt::Key_Left;
    if (key == "right") return Qt::Key_Right;
    if (key == "enter") return Qt::Key_Enter;
    if (key == "backspace") return Qt::Key_Backspace;
    if (key == "sysreq") return Qt::Key_SysReq;
    if (key == "escape") return Qt::Key_Escape;

    return Qt::Key_No;
}

SDL_GameControllerButton BindsSettingsModel::buttonStringToButton(QString button) const
{
    if (button == "x") return SDL_CONTROLLER_BUTTON_X;
    if (button == "y") return SDL_CONTROLLER_BUTTON_X;
    if (button == "a") return SDL_CONTROLLER_BUTTON_X;
    if (button == "b") return SDL_CONTROLLER_BUTTON_X;
    if (button == "left_button") return SDL_CONTROLLER_BUTTON_LEFTSHOULDER;
    if (button == "right_button") return SDL_CONTROLLER_BUTTON_RIGHTSHOULDER;
    if (button == "dpad_up") return SDL_CONTROLLER_BUTTON_DPAD_UP;
    if (button == "dpad_down") return SDL_CONTROLLER_BUTTON_DPAD_DOWN;
    if (button == "dpad_left") return SDL_CONTROLLER_BUTTON_DPAD_LEFT;
    if (button == "dpad_right") return SDL_CONTROLLER_BUTTON_DPAD_RIGHT;
    if (button == "left_stick") return SDL_CONTROLLER_BUTTON_LEFTSTICK;
    if (button == "right_stick") return SDL_CONTROLLER_BUTTON_RIGHTSTICK;

    return SDL_CONTROLLER_BUTTON_INVALID;
}

BindsSettingsModel::Action BindsSettingsModel::getActionForKey(Qt::Key key)
{
    return _keyActions.value(key);
}

BindsSettingsModel::Action BindsSettingsModel::getActionForButton(SDL_GameControllerButton button)
{
    return _buttonActions.value(button);
}

} // namespace Soro
