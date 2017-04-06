#include "mainwindowcontroller.h"
#include "maincontroller.h"
#include "qquickgstreamersurface.h"
#include "libsoromc/constants.h"

#include <QQmlComponent>

#include <Qt5GStreamer/QGst/ElementFactory>

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

    // Setup the camera views in the UI
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
        // Show no video pattern
        stopVideo(cameras[i].id);
    }

    _window->setProperty("selectedView", "video0");

    // Connect to gamepad events
    connect(MainController::getGamepadController(), SIGNAL(buttonPressed(SDL_GameControllerButton,bool)),
            this, SLOT(onGamepadButtonPressed(SDL_GameControllerButton,bool)));
}

void MainWindowController::clearVideo(int cameraId)
{
    int cameraIndex = MainController::getCameraSettingsModel()->getCameraIndexById(cameraId);
    if (!_videoPipelines[cameraIndex].isNull()) {
        _videoPipelines[cameraIndex]->setState(QGst::StateNull);
        _videoPipelines[cameraIndex].clear();
        _videoBins[cameraIndex].clear();
        _videoSinks[cameraIndex].clear();
    }
}

void MainWindowController::stopVideo(int cameraId, QString pattern)
{
    clearVideo(cameraId);
    int cameraIndex = MainController::getCameraSettingsModel()->getCameraIndexById(cameraId);
    QGst::PipelinePtr pipeline = QGst::Pipeline::create(QString("camera%1Pipeline").arg(cameraId).toLatin1().constData());
    QGst::BinPtr source = QGst::Bin::fromDescription(QString("videotestsrc pattern=%1 ! video/x-raw,width=640,height=480 ! videoconvert").arg(pattern));
    QGst::ElementPtr sink = QGst::ElementFactory::make("qt5videosink");
    QQuickGStreamerSurface *surface = qvariant_cast<QQuickGStreamerSurface*>(_window->property(QString("video%1Surface").arg(cameraIndex).toLatin1().constData()));

    _videoPipelines[cameraIndex] = pipeline;
    _videoBins[cameraIndex] = source;
    _videoSinks[cameraIndex] = sink;

    pipeline->add(source, sink);
    source->link(sink);
    surface->setSink(sink);
    pipeline->setState(QGst::StatePlaying);
}

void MainWindowController::playVideo(int cameraId, VideoFormat format)
{
    clearVideo(cameraId);
    int cameraIndex = MainController::getCameraSettingsModel()->getCameraIndexById(cameraId);
    QGst::PipelinePtr pipeline = QGst::Pipeline::create(QString("camera%1Pipeline").arg(cameraId).toLatin1().constData());
    QGst::BinPtr source = QGst::Bin::fromDescription("udpsrc port=" + QString::number(SORO_MC_FIRST_VIDEO_PORT + cameraIndex) + " ! " + format.createGstEncodingArgs());
    QGst::ElementPtr sink = QGst::ElementFactory::make("qt5videosink");
    QQuickGStreamerSurface *surface = qvariant_cast<QQuickGStreamerSurface*>(_window->property(QString("video%1Surface").arg(cameraIndex).toLatin1().constData()));

    _videoPipelines[cameraIndex] = pipeline;
    _videoBins[cameraIndex] = source;
    _videoSinks[cameraIndex] = sink;

    pipeline->add(source, sink);
    source->link(sink);
    surface->setSink(sink);
    pipeline->setState(QGst::StatePlaying);
}

void MainWindowController::notify(MessageType type, QString message)
{

}

void MainWindowController::notifyAll(MessageType type, QString message) {

}

void MainWindowController::onGamepadButtonPressed(SDL_GameControllerButton button, bool pressed)
{
    if (pressed)
    {
        switch (button)
        {
        case SDL_CONTROLLER_BUTTON_Y:
            if (_window->property("sidebarState").toString() == "visible")
            {
                _window->setProperty("sidebarState", "hidden");
            }
            else
            {
                _window->setProperty("sidebarState", "visible");
            }
            break;
        default:
            break;
        }
    }
}

} // namespace Soro
