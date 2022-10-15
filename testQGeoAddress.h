#pragma once
// standard C++ header:
#include <iostream>
#include <string>

// Qt header:
#include <QApplication>
#include <QGeoAddress>
#include <QGeoCodingManager>
#include <QGeoCoordinate>
#include <QGeoLocation>
#include <QGeoServiceProvider>

using namespace std;

int main(int argc, char** argv)
{
    cout << "Qt Version: " << QT_VERSION_STR << endl;
    // main application
#undef qApp // undef macro qApp out of the way
    QCoreApplication qApp(argc, argv);
    // check for available services
    QStringList qGeoSrvList
        = QGeoServiceProvider::availableServiceProviders();
    for (QString entry : qGeoSrvList) {
        cout << "Try service: " << entry.toStdString() << endl;
        // choose provider
        QGeoServiceProvider qGeoService(entry);
        QGeoCodingManager* pQGeoCoder = qGeoService.geocodingManager();
        if (!pQGeoCoder) {
            cerr
                << "GeoCodingManager '" << entry.toStdString()
                << "' not available!" << endl;
            continue;
        }
        QLocale qLocaleC(QLocale::C, QLocale::AnyCountry);
        pQGeoCoder->setLocale(qLocaleC);
        // build address
        QGeoAddress qGeoAddr;
        qGeoAddr.setCountry(QString::fromUtf8("Germany"));
        qGeoAddr.setPostalCode(QString::fromUtf8("88250"));
        qGeoAddr.setCity(QString::fromUtf8("Weingarten"));
        qGeoAddr.setStreet(QString::fromUtf8("Heinrich-Hertz-Str. 6"));
        QGeoCodeReply* pQGeoCode = pQGeoCoder->geocode(qGeoAddr);
        if (!pQGeoCode) {
            cerr << "GeoCoding totally failed!" << endl;
            continue;
        }
        cout << "Searching..." << endl;
        QObject::connect(pQGeoCode, &QGeoCodeReply::finished,
            [&qApp, &qGeoAddr, pQGeoCode]() {
                cout << "Reply: " << pQGeoCode->errorString().toStdString() << endl;
                switch (pQGeoCode->error()) {
#define CASE(ERROR) \
case QGeoCodeReply::ERROR: cerr << #ERROR << endl; break
                    CASE(NoError);
                    CASE(EngineNotSetError);
                    CASE(CommunicationError);
                    CASE(ParseError);
                    CASE(UnsupportedOptionError);
                    CASE(CombinationError);
                    CASE(UnknownError);
#undef CASE
                default: cerr << "Undocumented error!" << endl;
                }
                if (pQGeoCode->error() != QGeoCodeReply::NoError) return;
                // eval. result
                QList<QGeoLocation> qGeoLocs = pQGeoCode->locations();
                cout << qGeoLocs.size() << " location(s) returned." << endl;
                for (QGeoLocation& qGeoLoc : qGeoLocs) {
                    qGeoLoc.setAddress(qGeoAddr);
                    QGeoCoordinate qGeoCoord = qGeoLoc.coordinate();
                    cout
                        << "Lat.:  " << qGeoCoord.latitude() << endl
                        << "Long.: " << qGeoCoord.longitude() << endl
                        << "Alt.:  " << qGeoCoord.altitude() << endl;
                }
                qApp.exit(0);
            });
        return qApp.exec();
    }
    return 0;
}