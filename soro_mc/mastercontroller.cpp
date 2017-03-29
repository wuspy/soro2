#include "mastercontroller.h"
#include "maincontroller.h"

#include <QCoreApplication>

#define LOG_TAG "MasterController"

namespace Soro {

MasterController::MasterController(QObject *parent) : QObject(parent)
{
    _masterProcess.start(QCoreApplication::applicationDirPath() + "/soro_mc_master");
    connect(&_masterProcess, SIGNAL(started()), this, SLOT(processStarted()));
    connect(&_masterProcess, SIGNAL(finished(int)), this, SLOT(processFinished(int)));
}

MasterController::~MasterController()
{
    if (_masterProcess.state() != QProcess::NotRunning) {
        disconnect(&_masterProcess, SIGNAL(finished(int)), this, SLOT(processFinished(int)));
        _masterProcess.terminate();
        _masterProcess.waitForFinished();
    }
}

void MasterController::processStarted()
{
    MainController::logInfo(LOG_TAG, "Master process is running");
}

void MasterController::processFinished(int exitCode)
{
    MainController::panic(QString("Master process exited prematurely with exit code %1").arg(exitCode));
}

} // namespace Soro
