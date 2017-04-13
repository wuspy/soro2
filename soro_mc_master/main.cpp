#include <QCoreApplication>
#include <roscorecontroller.h>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);


    RosCoreController c;

    return a.exec();
}
