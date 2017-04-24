#ifndef BINDSSETTINGSMODEL_H
#define BINDSSETTINGSMODEL_H

#include <QString>
#include <QMap>
#include <QList>

#include <SDL2/SDL.h>

namespace Soro {

/* Settings loader for the keymap/buttonmap configuration. This file is a JSON file
 * specifying which keys and gamepad buttons are bound to what actions.
 */
class BindsSettingsModel
{
    Q_GADGET
public:
    /* Represents a type of action that can be bound to a key/button
     */
    enum ActionType
    {
        Action_Null = 0,
        Action_Toggle_Fullscreen,
        Action_Video_Off,
        Action_Video_On,
        Action_Audio_Off,
        Action_Audio_On
    };

    /* Abstract base struct for action arguments, inherited
     * by argument structs for various actions.
     */
    struct ActionArgs { };

    struct VideoOffActionArgs : public ActionArgs
    {
        int camera;
    };

    struct VideoOnActionArgs : public ActionArgs
    {
        uint camera;
        int profile;
    };

    struct AudioOnActionArgs : public ActionArgs
    {
        int profile;
    };

    /* Represents all the information about a key/button action, including
     * the type of action to be performed, and the arguments associated with
     * that action.
     */
    struct Action
    {
        ActionType type;
        ActionArgs *args;

        Action()
        {
            type = Action_Null;
            args = nullptr;
        }
    };

    /* Loads the media profile definitions from the settings file. This will
     * throw an exception of type QString if an error occurrs
     */
    void load();

    /* Gets the action bound to a key
     */
    BindsSettingsModel::Action getActionForKey(Qt::Key key);
    /* Gets the action bound to a gamepad button
     */
    BindsSettingsModel::Action getActionForButton(SDL_GameControllerButton button);

private:
    Qt::Key keyStringToKey(QString key) const;
    SDL_GameControllerButton buttonStringToButton(QString button) const;
    void parseAction(QJsonValue jsonObject, Action action);

    QHash<Qt::Key, Action> _keyActions;
    QHash<SDL_GameControllerButton, Action> _buttonActions;
};

} // namespace Soro

#endif // BINDSSETTINGSMODEL_H
