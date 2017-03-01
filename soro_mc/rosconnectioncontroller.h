#ifndef ROSCONNECTIONCONTROLLER_H
#define ROSCONNECTIONCONTROLLER_H

#include <QObject>
#include <QUdpSocket>

namespace Soro {

class RosConnectionController : public QObject
{
    Q_OBJECT
public:
    explicit RosConnectionController(QObject *parent = 0);

signals:
    void rosInitialized();

private:
    void initRos();
    QUdpSocket *_udpSocket;

private slots:
    void udpReadyRead();
};

} // namespace Soro

#endif // ROSCONNECTIONCONTROLLER_H
