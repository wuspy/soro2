#include <QCoreApplication>
#include "maincontroller.h"

using namespace Soro;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    MainController::init(&a);
    return a.exec();
}
