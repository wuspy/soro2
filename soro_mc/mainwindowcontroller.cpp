#include "mainwindowcontroller.h"
#include "maincontroller.h"
#include "qquickgstreamersurface.h"
#include "libsoromc/constants.h"
#include "libsoromc/logger.h"

#include <QQmlComponent>

#include <Qt5GStreamer/QGst/ElementFactory>

#define LogTag "MainWindowController"

namespace Soro {

MainWindowController::MainWindowController(QQmlEngine *engine, QObject *parent) : QObject(parent)
{
    QQmlComponent qmlComponent(engine, QUrl("qrc:/main.qml"));
    _window = qobject_cast<QQuickWindow*>(qmlComponent.create());
    if (!qmlComponent.errorString().isEmpty() || !_window)
    {
        // There was an error creating the QML window. This is most likely due to a QML syntax error
        throw QString("Failed to create QML window:  %1").arg(qmlComponent.errorString());
    }
    //
    // Setup the camera views in the UI
    //
    int videoCount = MainController::getCameraSettingsModel()->getCameraCount();
    if (videoCount > 10)
    {
        MainController::panic("UI does not support more than 10 different camera views");
        return;
    }
    _window->setProperty("videoCount", videoCount);

    QList<CameraSettingsModel::Camera> cameras = MainController::getCameraSettingsModel()->getCameras();
    for (int i = 0; i < cameras.count(); i++)
    {
        // Set camera name
        _window->setProperty(QString("video%1Name").arg(i).toLatin1().constData(), cameras[i].name);
        // Set hardware rendering flag
        qvariant_cast<QQuickGStreamerSurface*>(_window->property(QString("video%1Surface").arg(i).toLatin1().constData()))
                ->setEnableHardwareRendering(MainController::getSettingsModel()->getEnableHwRendering());
        // Show no video pattern
        stopVideo(cameras[i].id);
    }

    _window->setProperty("selectedView", "video0");

    //
    // Setup ROS communication
    //

    Logger::logInfo(LogTag, "Creating ROS publisher and subscriber for notificaiton topic...");
    _notifyPublisher = MainController::getNodeHandle()->advertise<message_gen::notification>("notification", 10);
    _notifySubscriber = MainController::getNodeHandle()->subscribe
            <message_gen::notification, Soro::MainWindowController>
            ("notification", 10, &MainWindowController::onNewNotification, this);
    Logger::logInfo(LogTag, "ROS publisher and subscriber created");

    // Connect to gamepad events
    connect(MainController::getGamepadController(), &GamepadController::buttonPressed,
            this, &MainWindowController::onGamepadButtonPressed);

    // Connect to connection events
    connect(MainController::getConnectionStatusController(), &ConnectionStatusController::bitrateUpdate,
            this, &MainWindowController::onBitrateUpdated);
    connect(MainController::getConnectionStatusController(), &ConnectionStatusController::latencyUpdate,
            this, &MainWindowController::onLatencyUpdated);
    connect(MainController::getConnectionStatusController(), &ConnectionStatusController::connectedChanged,
            this, &MainWindowController::onConnectedChanged);
}

void MainWindowController::clearVideo(int cameraId)
{
    int cameraIndex = MainController::getCameraSettingsModel()->getCameraIndexById(cameraId);
    if (!_videoPipelines[cameraIndex].isNull()) {
        _videoPipelines[cameraIndex]->setState(QGst::StateNull);
        _videoPipelines[cameraIndex].clear();
        _videoBins[cameraIndex].clear();
    }
}

void MainWindowController::stopVideo(int cameraId, QString pattern)
{
    clearVideo(cameraId);
    int cameraIndex = MainController::getCameraSettingsModel()->getCameraIndexById(cameraId);
    QGst::PipelinePtr pipeline = QGst::Pipeline::create(QString("camera%1Pipeline").arg(cameraId).toLatin1().constData());
    QGst::BinPtr source = QGst::Bin::fromDescription(QString("videotestsrc pattern=%1 ! video/x-raw,width=640,height=480 ! videoconvert").arg(pattern));
    QQuickGStreamerSurface *surface = qvariant_cast<QQuickGStreamerSurface*>(_window->property(QString("video%1Surface").arg(cameraIndex).toLatin1().constData()));

    _videoPipelines[cameraIndex] = pipeline;
    _videoBins[cameraIndex] = source;

    pipeline->add(source, surface->videoSink());
    source->link(surface->videoSink());
    pipeline->setState(QGst::StatePlaying);
}

void MainWindowController::playVideo(int cameraId, VideoFormat format)
{
    clearVideo(cameraId);
    int cameraIndex = MainController::getCameraSettingsModel()->getCameraIndexById(cameraId);
    QGst::PipelinePtr pipeline = QGst::Pipeline::create(QString("camera%1Pipeline").arg(cameraId).toLatin1().constData());
    QGst::BinPtr source = QGst::Bin::fromDescription("udpsrc port=" + QString::number(SORO_MC_FIRST_VIDEO_PORT + cameraIndex) + " ! " + format.createGstEncodingArgs());
    QQuickGStreamerSurface *surface = qvariant_cast<QQuickGStreamerSurface*>(_window->property(QString("video%1Surface").arg(cameraIndex).toLatin1().constData()));

    _videoPipelines[cameraIndex] = pipeline;
    _videoBins[cameraIndex] = source;

    pipeline->add(source, surface->videoSink());
    source->link(surface->videoSink());
    pipeline->setState(QGst::StatePlaying);
}

void MainWindowController::onNewNotification(message_gen::notification msg)
{
    // Show this message in the UI
    QString title = QString(msg.title.c_str());
    QString message = QString(msg.message.c_str());
    notify((int)msg.type, title, message);

    switch (msg.type)
    {
    case NOTIFICATION_TYPE_ERROR:
        Logger::logError(LogTag, QString("Received error notification: %1 - %2 ").arg(title, message));
        break;
    case NOTIFICATION_TYPE_WARNING:
        Logger::logWarn(LogTag, QString("Received warning notification: %1 - %2 ").arg(title, message));
        break;
    case NOTIFICATION_TYPE_INFO:
        Logger::logInfo(LogTag, QString("Received info notification: %1 - %2 ").arg(title, message));
        break;
    }
}

void MainWindowController::notify(int type, QString title, QString message)
{
    //TODO
}

void MainWindowController::notifyAll(int type, QString title, QString message)
{
    message_gen::notification msg;
    msg.type = (uint8_t)type;
    msg.title = title.toStdString();
    msg.message = message.toStdString();

    // Publish this notification on the notification topic. We will get this message back,
    // since we are also subscribed to it, and that's when we'll show it from the onNewNotification() function
    _notifyPublisher.publish(msg);
}

void MainWindowController::onConnectedChanged(bool connected)
{
    _window->setProperty("connected", connected);
}

void MainWindowController::onLatencyUpdated(quint32 latency)
{
    _window->setProperty("latency", latency);
}

void MainWindowController::onBitrateUpdated(quint64 bitsUp, quint64 bitsDown)
{
    _window->setProperty("bitrateUp", bitsUp);
    _window->setProperty("bitrateDown", bitsDown);
}

void MainWindowController::onGamepadButtonPressed(SDL_GameControllerButton button, bool pressed)
{
    if (pressed)
    {
        switch (button)
        {
        case SDL_CONTROLLER_BUTTON_Y:
            //
            // Toggle sidebar
            //
            QMetaObject::invokeMethod(_window, "toggleSidebar");
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_UP:
            //
            // Move to view above
            //
            QMetaObject::invokeMethod(_window, "selectViewAbove");
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
            //
            // Move to view below
            //
            QMetaObject::invokeMethod(_window, "selectViewBelow");
            break;
        default:
            break;
        }
    }
}

} // namespace Soro
