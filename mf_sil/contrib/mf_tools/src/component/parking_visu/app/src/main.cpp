#define WIN64_LEAN_AND_MEAN
#include "plannervisualization.h"
#include <QApplication>
#include <QStyleFactory>
#include <QCommandLineParser>
#include <QDebug>
#include <QSharedPointer>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "clienthandler.h"
#include <parkingvisuclient.h>


void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    std::cout << msg.toStdString() << std::endl;
}

int main(int argc, char *argv[])
{
    QCoreApplication::addLibraryPath(".");
    QApplication app(argc, argv);

    // options for command line arguments starting the app without gui
    QCommandLineParser parser;
    QCommandLineOption grabOption(QStringList() << "g" << "grab", "Grab screenshot of scene <jsonfile> and exit.", ".json file");
    parser.addOption(grabOption);
    QCommandLineOption outputOption(QStringList() << "o" << "out", "Image output path for -g.", ".png file");
    parser.addOption(outputOption);
    QCommandLineOption clientOption(QStringList() << "c" << "client", "Start in client mode.");
    parser.addOption(clientOption);
    QCommandLineOption widthOption(QStringList() << "w" << "width", "Image width for -g.", "width", "1400");
    parser.addOption(widthOption);
    QCommandLineOption heightOption(QStringList() << "h" << "height", "Image height for -g.", "height", "1050");
    parser.addOption(heightOption);
    QCommandLineOption parameterPathOptionTrjpla(QStringList() << "pTrjPla" << "pathTrjPla", "TRJPLA Parameters directory for -g.", "pathTrjPla", "D:/GitHub/mf_trjpla/src/platform/data/AP_Sim");
    parser.addOption(parameterPathOptionTrjpla);
    QCommandLineOption parameterPathOptionSysFunc(QStringList() << "pSys" << "pathSysFunc", "Sys Func Parameters directory for -g.", "pathSysFunc", "D:/GitHub/mf_common/src/platform/data");
    parser.addOption(parameterPathOptionSysFunc);
    QCommandLineOption parameterPathOptionVeh(QStringList() << "pVeh" << "pathVeh", "Vehicle Parameters directory for -g.", "pathVeh", "D:/GitHub/mf_common/src/platform/data/AP_Sim");
    parser.addOption(parameterPathOptionVeh);
    parser.process(app);


    QSharedPointer<PlannerVisualization> visu;

    if(parser.isSet(clientOption)) {
        qInstallMessageHandler(myMessageOutput);
        qDebug() << "starting in client mode";
        auto client = new ParkingVisuClient(&app);
        client->connectToServer();
    } else {
        app.setStyle(QStyleFactory::create("Fusion"));

        /* Grab feature allowing for loading an EM and making a screenshot. This is currently used for 
            regression tests on L3 in mf_sil for creating pics for the reports of parking scenarios */
        if(parser.isSet(grabOption)) {
            visu = QSharedPointer<PlannerVisualization>(new PlannerVisualization(false));
            visu->setAttribute(Qt::WA_DontShowOnScreen);
            visu->show();

            int width = parser.value(widthOption).toInt();
            int height = parser.value(heightOption).toInt();

            const auto jsonPath = parser.value(grabOption);
            const auto outValues = parser.values(outputOption);
            const QString outPath = (outValues.empty() || outValues.at(0).isEmpty())? jsonPath + ".png" : outValues.at(0);
            const auto paramPathTrjpla = parser.value(parameterPathOptionTrjpla);
            const auto paramPathSysFunc = parser.value(parameterPathOptionSysFunc);
            const auto paramPathVeh = parser.value(parameterPathOptionVeh);

            qDebug() << "Grabbing screenshot" << width << "x" << height;
            qDebug() << ".json:" << jsonPath;
            qDebug() << ".png:" << outPath;
            qDebug() << "Trjpla Params:" << paramPathTrjpla;
            qDebug() << "Sys Func Params:" << paramPathSysFunc;
            qDebug() << "Vehicle Params:" << paramPathVeh;

            auto errMsg = visu->grabJSONScene(jsonPath, outPath, width, height, paramPathTrjpla, paramPathSysFunc, paramPathVeh);
            if(errMsg.isEmpty()) {
                return 0;
            } else {
                std::cerr << "Error: " << errMsg.toStdString() << std::endl;
                return -1;
            }
        }

        qDebug() << "starting in server mode";
        visu = QSharedPointer<PlannerVisualization>(new PlannerVisualization());
        visu->show();
    }

    return app.exec();
}
