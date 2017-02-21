/*#include <QGuiApplication>
#include <QQmlApplicationEngine>

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    engine.load(QUrl(QLatin1String("qrc:/main2.qml")));

    return app.exec();
}
*/

#include "player.h"
#include <QtGui/QGuiApplication>
#include <QtQuick/QQuickView>
#include <QtQml/QQmlContext>
#include <QtQml/QQmlEngine>
#include <Qt5GStreamer/QGst/Init>
#include <Qt5GStreamer/QGst/Quick/VideoItem>
#include <Qt5GStreamer/QGst/Quick/VideoSurface>

int main(int argc, char **argv)
{
    QGuiApplication app(argc, argv);
    QGst::init(&argc, &argv);

    // Register Qt5GStreamer with QML
    //qmlRegisterType<QGst::Quick::VideoItem>("QtGStreamer", 1, 0, "VideoItem");
    //qmlRegisterType<Player>("QtGStreamer", 1, 0, "Player");
    //qmlRegisterUncreatableType<QGst::Quick::VideoSurface>("QtGStreamer", 1, 0, "VideoSurface");

    QQuickView view;

    QGst::Quick::VideoSurface *surface = new QGst::Quick::VideoSurface;

    view.rootContext()->setContextProperty(QLatin1String("videoSurface1"), surface);

    Player *player = new Player(&view);
    player->setVideoSink(surface->videoSink());
    if (argc > 1)
        player->setUri(QString::fromLocal8Bit(argv[1]));
    else
        player->setUri(QLatin1Literal("file:///home/soro/Downloads/big_buck_bunny_480p_surround-fix.avi"));
    view.rootContext()->setContextProperty(QLatin1String("player"), player);

    view.setSource(QUrl(QLatin1String("qrc:///video.qml")));
    view.show();

    return app.exec();
}
