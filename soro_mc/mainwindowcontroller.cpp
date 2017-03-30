#include "mainwindowcontroller.h"
#include "maincontroller.h"
#include "qquickgstreamersurface.h"

#include <QQmlComponent>

#include <Qt5GStreamer/QGst/Pipeline>
#include <Qt5GStreamer/QGst/Element>
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
    _window->setProperty("cameraCount", MainController::getCameraSettingsModel()->getCameraCount());
    QList<CameraSettingsModel::Camera> cameras = MainController::getCameraSettingsModel()->getCameras();
    for (int i = 0; i < cameras.count(); i++)
    {
        QQuickGStreamerSurface *vidSurface = qvariant_cast<QQuickGStreamerSurface*>(_window->property(QString("cameraThumbnail%1GstreamerSurface").arg(i).toLatin1().constData()));

        ///////////////////////////
        //////  TESTING  //////////

        QGst::PipelinePtr pipeline = QGst::Pipeline::create("pipeline1");
        QGst::BinPtr source = QGst::Bin::fromDescription("videotestsrc pattern=snow ! videoconvert");
        QGst::ElementPtr sink = QGst::ElementFactory::make("qt5videosink");
        pipeline->add(source, sink);
        source->link(sink);
        vidSurface->setSink(sink);
        pipeline->setState(QGst::StatePlaying);

        _window->setProperty(QString("camera%1Name").arg(i).toLatin1().constData(), cameras[i].name);
    }

    QQuickGStreamerSurface *vidSurface = qvariant_cast<QQuickGStreamerSurface*>(_window->property("mainGstreamerSurface"));
    QGst::PipelinePtr pipeline = QGst::Pipeline::create("pipeline1");
    QGst::BinPtr source = QGst::Bin::fromDescription("videotestsrc pattern=snow ! videoconvert");
    QGst::ElementPtr sink = QGst::ElementFactory::make("qt5videosink");
    pipeline->add(source, sink);
    source->link(sink);
    vidSurface->setSink(sink);
    pipeline->setState(QGst::StatePlaying);

    _window->setProperty("selectedViewIndex", 0);

    // Connect to gamepad events
    connect(MainController::getGamepadController(), SIGNAL(buttonPressed(SDL_GameControllerButton,bool)),
            this, SLOT(onGamepadButtonPressed(SDL_GameControllerButton,bool)));
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
        case SDL_CONTROLLER_BUTTON_DPAD_UP:
        {
            int newIndex = _window->property("selectedViewIndex").toInt() - 1;
            if (newIndex >= 0)
            {
                _window->setProperty("selectedViewIndex", newIndex);
            }
        }
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
        {
            int newIndex = _window->property("selectedViewIndex").toInt() + 1;
            if (newIndex < MainController::getCameraSettingsModel()->getCameraCount() + 1)
            {
                _window->setProperty("selectedViewIndex", newIndex);
            }
        }
            break;
        }
    }
}

} // namespace Soro
