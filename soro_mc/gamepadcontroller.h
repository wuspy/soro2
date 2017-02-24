#ifndef GAMEPADCONTROLLER_H
#define GAMEPADCONTROLLER_H

#include <QObject>

class GamepadController : public QObject
{
    Q_OBJECT
public:
    explicit GamepadController(QObject *parent = 0);

signals:

public slots:
};

#endif // GAMEPADCONTROLLER_H