#include <QCoreApplication>

#include "maincontroller.h"

using namespace Soro;

int main(int argc, char *argv[])
{
    QCoreApplication::setOrganizationName("Sooner Rover");
    QCoreApplication::setOrganizationDomain("ou.edu/soonerrover");
    QCoreApplication::setApplicationName("Audio Server");
    QCoreApplication app(argc, argv);

    MainController::init(&app);

    return app.exec();
}
