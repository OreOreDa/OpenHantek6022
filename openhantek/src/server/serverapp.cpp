// SPDX-License-Identifier: GPL-2.0-or-later

#include "serverapp.h"

#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDebug>
#include <QElapsedTimer>
#include <QThread>
#include <iostream>
#include <memory>

#ifdef Q_OS_FREEBSD
#include <libusb.h>
#define libusb_setlocale( x ) (void)0
#else
#include <libusb-1.0/libusb.h>
#endif

// Settings
#include "dsosettings.h"

// DSO core logic
#include "capturing.h"
#include "dsomodel.h"
#include "hantekdsocontrol.h"
#include "usb/devicereconnectionsupervisor.h"
#include "usb/finddevices.h"
#include "usb/scopedevice.h"
#include "usb/uploadFirmware.h"

// Post processing
#include "post/graphgenerator.h"
#include "post/postprocessing.h"
#include "post/spectrumgenerator.h"

// Exporter
#include "exporting/exportcsv.h"
#include "exporting/exporterprocessor.h"
#include "exporting/exporterregistry.h"
#include "exporting/exportjson.h"

// Bluetooth RFCOMM server
#include "bluetoothserver.h"

// WiFi (TCP/IP) server
#include "wifiserver.h"

#include "OH_VERSION.h"

// verboseLevel is defined in main.cpp
extern int verboseLevel;


/// \brief Discover and open a scope device without any GUI.
///
/// Polls libusb, uploads firmware to devices that need it and returns the first
/// device that can be connected. Mirrors the logic of DevicesListModel but
/// headless. Returns nullptr on timeout.
static std::unique_ptr< ScopeDevice > acquireDeviceHeadless( libusb_context *context, int verbose, int timeoutMs ) {
    FindDevices findDevices( context, verbose );
    QElapsedTimer timer;
    timer.start();
    while ( timer.elapsed() < timeoutMs ) {
        findDevices.updateDeviceList();
        const FindDevices::DeviceList *devices = findDevices.getDevices();
        for ( auto &entry : *devices ) {
            ScopeDevice *device = entry.second.get();
            if ( device->needsFirmware() ) {
                UploadFirmware uf;
                if ( verbose )
                    qDebug() << "server: uploading firmware to" << device->getModel()->name;
                if ( !uf.startUpload( device ) )
                    qWarning() << "server: firmware upload failed:" << uf.getErrorMessage();
                // Device re-enumerates; it will reappear on a later poll iteration.
                continue;
            }
            QString errorMessage;
            if ( device->connectDevice( errorMessage ) ) {
                device->disconnectFromDevice(); // reopened by the caller via connectDevice()
                if ( verbose )
                    qDebug() << "server: found device" << device->getModel()->name;
                return findDevices.takeDevice( entry.first );
            }
        }
        QThread::msleep( 200 );
    }
    return nullptr;
}


int runHeadlessServer( int argc, char **argv ) {
    QElapsedTimer startupTime;
    startupTime.start();

    QCoreApplication app( argc, argv );
    QCoreApplication::setOrganizationName( "OpenHantek" );
    QCoreApplication::setOrganizationDomain( "openhantek.org" );
    QCoreApplication::setApplicationName( "OpenHantek6022" );
    QCoreApplication::setApplicationVersion( VERSION );

    //////// Parse the subset of options relevant for the headless server ////////
    QCommandLineParser p;
    p.addHelpOption();
    p.addVersionOption();
    QCommandLineOption serverOption( { "S", "server" }, "Run headless server (no GUI): Bluetooth RFCOMM + WiFi TCP" );
    p.addOption( serverOption );
    QCommandLineOption demoModeOption( { "d", "demoMode" }, "Demo mode without scope HW" );
    p.addOption( demoModeOption );
    QCommandLineOption noAutoConnectOption( "noAutoConnect", "Do not connect automatically" );
    p.addOption( noAutoConnectOption );
    QCommandLineOption configFileOption( { "c", "config" }, "Load config file", "File" );
    p.addOption( configFileOption );
    QCommandLineOption verboseOption( "verbose", "Verbose tracing", "Level" );
    p.addOption( verboseOption );
    QCommandLineOption portOption(
        { "p", "port" },
        QString( "TCP port for the WiFi control server (default = %1)" ).arg( WifiServer::DEFAULT_PORT ), "Port" );
    p.addOption( portOption );
    p.process( app );

    const bool demoMode = p.isSet( demoModeOption );
    if ( p.isSet( verboseOption ) )
        verboseLevel = p.value( "verbose" ).toInt();
    const QString configFileName = p.isSet( configFileOption ) ? p.value( "config" ) : QString();
    quint16 tcpPort = WifiServer::DEFAULT_PORT;
    if ( p.isSet( portOption ) )
        tcpPort = quint16( p.value( "port" ).toUInt() );

    if ( verboseLevel )
        qDebug() << "OpenHantek6022 headless server - version" << VERSION;

    //////// Find / open a scope device (headless) ////////
    libusb_context *context = nullptr;
    std::unique_ptr< ScopeDevice > scopeDevice;

    if ( !demoMode ) {
#if ( LIBUSB_API_VERSION >= 0x0100010A )
        int error = libusb_init_context( &context, NULL, 0 );
#else
        int error = libusb_init( &context );
#endif
        if ( error ) {
            qCritical() << "server: can't initialize USB:" << libUsbErrorString( error );
            return -1;
        }
        scopeDevice = acquireDeviceHeadless( context, verboseLevel, 30000 );
        if ( !scopeDevice ) {
            qCritical() << "server: no compatible device found";
            libusb_exit( context );
            return -1;
        }
        QString errorMessage;
        if ( !scopeDevice->connectDevice( errorMessage ) ) {
            qCritical() << "server: cannot connect to device:" << errorMessage;
            libusb_exit( context );
            return -1;
        }
    } else {
        scopeDevice = std::unique_ptr< ScopeDevice >( new ScopeDevice() );
    }

    const DSOModel *model = scopeDevice->getModel();
    if ( verboseLevel )
        qDebug() << "server: use device" << model->name << "serial" << scopeDevice->getSerialNumber();

    //////// DSO control object in its own thread ////////
    QThread dsoControlThread;
    dsoControlThread.setObjectName( "dsoControlThread" );
    HantekDsoControl dsoControl( scopeDevice.get(), model, verboseLevel );
    dsoControl.moveToThread( &dsoControlThread );
    QObject::connect( &dsoControlThread, &QThread::started, &dsoControl, &HantekDsoControl::stateMachine );

    std::unique_ptr< DeviceReconnectionSupervisor > reconnectSupervisor;
    if ( context && scopeDevice && scopeDevice->isRealHW() ) {
        reconnectSupervisor = std::unique_ptr< DeviceReconnectionSupervisor >(
            new DeviceReconnectionSupervisor( context, &dsoControl, scopeDevice, verboseLevel, &app ) );
        QObject::connect( QCoreApplication::instance(), &QCoreApplication::aboutToQuit, reconnectSupervisor.get(),
                          [ &reconnectSupervisor ]() { reconnectSupervisor->setClosing(); } );
        QObject::connect(
            &dsoControl, &HantekDsoControl::communicationError, reconnectSupervisor.get(),
            [ &reconnectSupervisor ]() { reconnectSupervisor->handleDeviceDisconnected( false ); }, Qt::QueuedConnection );
    }

    const Dso::ControlSpecification *spec = model->spec();

    //////// Settings ////////
    DsoSettings settings( scopeDevice.get(), verboseLevel, false );
    if ( !configFileName.isEmpty() )
        settings.loadFromFile( configFileName );

    //////// Exporters ////////
    ExporterRegistry exportRegistry( spec, &settings );
    ExporterCSV exporterCSV;
    ExporterJSON exporterJSON;
    ExporterProcessor samplesToExportRaw( &exportRegistry );
    exportRegistry.registerExporter( &exporterCSV );
    exportRegistry.registerExporter( &exporterJSON );

    //////// Post processing in its own thread ////////
    QThread postProcessingThread;
    postProcessingThread.setObjectName( "postProcessingThread" );
    PostProcessing postProcessing( settings.scope.countChannels(), verboseLevel );
    SpectrumGenerator spectrumGenerator( &settings.scope, &settings.analysis );
    GraphGenerator graphGenerator( &settings.scope, &settings.view );
    postProcessing.registerProcessor( &samplesToExportRaw );
    postProcessing.registerProcessor( &spectrumGenerator );
    postProcessing.registerProcessor( &graphGenerator );
    postProcessing.moveToThread( &postProcessingThread );
    QObject::connect( &dsoControl, &HantekDsoControl::samplesAvailable, &postProcessing, &PostProcessing::input );

    //////// Bluetooth RFCOMM server ////////
    BluetoothServer server( &dsoControl, verboseLevel );
    QObject::connect( &postProcessing, &PostProcessing::processingFinished, &server, &BluetoothServer::onProcessingFinished,
                      Qt::DirectConnection );
    // Apply the initial device settings, exactly like the GUI's settingsLoaded.
    QObject::connect(
        &postProcessing, &PostProcessing::processingFinished, &exportRegistry, &ExporterRegistry::input, Qt::DirectConnection );

    if ( !server.start() ) {
        qCritical() << "server: failed to start Bluetooth RFCOMM server";
        return -1;
    }

    //////// WiFi (TCP/IP) server ////////
    WifiServer wifiServer( &dsoControl, tcpPort, verboseLevel );
    QObject::connect( &postProcessing, &PostProcessing::processingFinished, &wifiServer, &WifiServer::onProcessingFinished,
                      Qt::DirectConnection );

    if ( !wifiServer.start() ) {
        qCritical() << "server: failed to start WiFi TCP server on port" << tcpPort;
        return -1;
    }

    //////// Start threads ////////
    dsoControl.enableSamplingUI();
    postProcessingThread.start();
    dsoControlThread.start();
    CapturingThread capturingThread( &dsoControl );
    capturingThread.start();

    // Push the current settings into the control object once the thread is up.
    QMetaObject::invokeMethod( &dsoControl, [ &dsoControl, &settings ]() { dsoControl.applySettings( &settings.scope ); },
                               Qt::QueuedConnection );

    if ( verboseLevel )
        qDebug() << startupTime.elapsed() << "ms: enter server event loop";
    int appStatus = app.exec();

    //////// Shutdown, mirroring main.cpp ////////
    if ( reconnectSupervisor )
        reconnectSupervisor->setClosing();
    server.stop();
    wifiServer.stop();

    dsoControl.quitSampling();
    unsigned waitForDso = unsigned( 2000 * dsoControl.getSamplesize() / dsoControl.getSamplerate() );
    waitForDso = qMax( waitForDso, 10000U );
    capturingThread.requestInterruption();
    capturingThread.wait( waitForDso );

    dsoControlThread.quit();
    dsoControlThread.wait( waitForDso );

    postProcessing.stop();
    postProcessingThread.quit();
    postProcessingThread.wait( 10000 );

    dsoControl.prepareForShutdown();

    if ( scopeDevice )
        scopeDevice.reset();
    if ( context )
        libusb_exit( context );

    return appStatus;
}

