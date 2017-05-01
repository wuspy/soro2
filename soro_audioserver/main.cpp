#include <QCoreApplication>

int main(int argc, char *argv[])
{
    QCoreApplication::setOrganizationName("Sooner Rover");
    QCoreApplication::setOrganizationDomain("ou.edu/soonerrover");
    QCoreApplication::setApplicationName("Audio Server");
    QCoreApplication a(argc, argv);

    return a.exec();
}
