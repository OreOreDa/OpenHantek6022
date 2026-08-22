// SPDX-License-Identifier: GPL-2.0-or-later

#include "bluetoothserver.h"

#include <QDebug>
#include <QReadLocker>
#include <QThread>

#include <QtBluetooth/QBluetoothLocalDevice>
#include <QtBluetooth/QBluetoothServer>
#include <QtBluetooth/QBluetoothSocket>

#include "hantekdso/dsosamples.h"
#include "hantekdso/enums.h"
#include "hantekdso/errorcodes.h"
#include "hantekdso/hantekdsocontrol.h"
#include "post/ppresult.h"

// protobuf generated header (built from openhantek/proto/openhantek.proto)
#include "openhantek.pb.h"

namespace proto = openhantek::proto;

// ---------------------------------------------------------------------------
// enum conversion helpers (proto <-> Dso)
// ---------------------------------------------------------------------------

static Dso::Coupling toDso( proto::Coupling c ) {
    switch ( c ) {
    case proto::COUPLING_AC:
        return Dso::Coupling::AC;
    case proto::COUPLING_GND:
        return Dso::Coupling::GND;
    case proto::COUPLING_DC:
    default:
        return Dso::Coupling::DC;
    }
}

static Dso::TriggerMode toDso( proto::TriggerMode m ) {
    switch ( m ) {
    case proto::TRIGGER_NORMAL:
        return Dso::TriggerMode::NORMAL;
    case proto::TRIGGER_SINGLE:
        return Dso::TriggerMode::SINGLE;
    case proto::TRIGGER_ROLL:
        return Dso::TriggerMode::ROLL;
    case proto::TRIGGER_AUTO:
    default:
        return Dso::TriggerMode::AUTO;
    }
}

static Dso::Slope toDso( proto::Slope s ) {
    switch ( s ) {
    case proto::SLOPE_NEGATIVE:
        return Dso::Slope::Negative;
    case proto::SLOPE_BOTH:
        return Dso::Slope::Both;
    case proto::SLOPE_POSITIVE:
    default:
        return Dso::Slope::Positive;
    }
}

static proto::ErrorCode toProto( Dso::ErrorCode e ) {
    switch ( e ) {
    case Dso::ErrorCode::CONNECTION:
        return proto::ERROR_CONNECTION;
    case Dso::ErrorCode::UNSUPPORTED:
        return proto::ERROR_UNSUPPORTED;
    case Dso::ErrorCode::PARAMETER:
        return proto::ERROR_PARAMETER;
    case Dso::ErrorCode::NONE:
    default:
        return proto::ERROR_NONE;
    }
}

// Human readable name of a command payload, for logging.
static const char *commandName( proto::Command::PayloadCase c ) {
    switch ( c ) {
    case proto::Command::kEnableSampling:
        return "EnableSampling";
    case proto::Command::kSetSamplerate:
        return "SetSamplerate";
    case proto::Command::kSetRecordTime:
        return "SetRecordTime";
    case proto::Command::kSetChannelUsed:
        return "SetChannelUsed";
    case proto::Command::kSetChannelInverted:
        return "SetChannelInverted";
    case proto::Command::kSetProbe:
        return "SetProbe";
    case proto::Command::kSetGain:
        return "SetGain";
    case proto::Command::kSetCoupling:
        return "SetCoupling";
    case proto::Command::kSetTriggerMode:
        return "SetTriggerMode";
    case proto::Command::kSetTriggerSource:
        return "SetTriggerSource";
    case proto::Command::kSetTriggerSmooth:
        return "SetTriggerSmooth";
    case proto::Command::kSetTriggerLevel:
        return "SetTriggerLevel";
    case proto::Command::kSetTriggerSlope:
        return "SetTriggerSlope";
    case proto::Command::kSetTriggerPosition:
        return "SetTriggerPosition";
    case proto::Command::kSetCalFreq:
        return "SetCalFreq";
    case proto::Command::kRestartSampling:
        return "RestartSampling";
    case proto::Command::kCalibrateOffset:
        return "CalibrateOffset";
    case proto::Command::kStringCommand:
        return "StringCommand";
    case proto::Command::PAYLOAD_NOT_SET:
    default:
        return "<none>";
    }
}

// Human readable name of a telemetry payload, for logging.
static const char *telemetryName( proto::Telemetry::PayloadCase c ) {
    switch ( c ) {
    case proto::Telemetry::kShowSamplingStatus:
        return "ShowSamplingStatus";
    case proto::Telemetry::kStatusMessage:
        return "StatusMessage";
    case proto::Telemetry::kSamplerateLimitsChanged:
        return "SamplerateLimitsChanged";
    case proto::Telemetry::kSamplerateSet:
        return "SamplerateSet";
    case proto::Telemetry::kSamplerateCalculated:
        return "SamplerateCalculated";
    case proto::Telemetry::kCommunicationError:
        return "CommunicationError";
    case proto::Telemetry::kDeviceAvailabilityChanged:
        return "DeviceAvailabilityChanged";
    case proto::Telemetry::kLiveCalibrationError:
        return "LiveCalibrationError";
    case proto::Telemetry::kSamples:
        return "Samples";
    case proto::Telemetry::kProcessedResult:
        return "ProcessedResult";
    case proto::Telemetry::PAYLOAD_NOT_SET:
    default:
        return "<none>";
    }
}

// ---------------------------------------------------------------------------
// construction / lifecycle
// ---------------------------------------------------------------------------

const QBluetoothUuid BluetoothServer::serviceUuid() {
    // Fixed custom UUID identifying the OpenHantek6022 RFCOMM service.
    return QBluetoothUuid( QStringLiteral( "6022feed-0000-1000-8000-00805f9b34fb" ) );
}

BluetoothServer::BluetoothServer( HantekDsoControl *dsoControl, int verboseLevel, QObject *parent )
    : QObject( parent ), dsoControl( dsoControl ), verboseLevel( verboseLevel ) {}

BluetoothServer::~BluetoothServer() { stop(); }

bool BluetoothServer::start() {
    if ( rfcommServer )
        return true;

    qInfo() << "BluetoothServer: starting RFCOMM server ...";
    rfcommServer = new QBluetoothServer( QBluetoothServiceInfo::RfcommProtocol, this );
    connect( rfcommServer, &QBluetoothServer::newConnection, this, &BluetoothServer::onNewConnection );

    if ( !rfcommServer->listen() ) {
        qCritical() << "BluetoothServer: cannot listen on RFCOMM (is Bluetooth powered on?)";
        delete rfcommServer;
        rfcommServer = nullptr;
        return false;
    }
    qInfo() << "BluetoothServer: listening on RFCOMM channel" << rfcommServer->serverPort();

    // Build and advertise the service record.
    QBluetoothServiceInfo::Sequence classId;
    classId << QVariant::fromValue( serviceUuid() );
    serviceInfo.setAttribute( QBluetoothServiceInfo::ServiceClassIds, classId );
    serviceInfo.setServiceUuid( serviceUuid() );
    serviceInfo.setAttribute( QBluetoothServiceInfo::ServiceName, QStringLiteral( "OpenHantek6022 RFCOMM" ) );
    serviceInfo.setAttribute( QBluetoothServiceInfo::ServiceDescription,
                              QStringLiteral( "Headless OpenHantek6022 oscilloscope control server" ) );
    serviceInfo.setAttribute( QBluetoothServiceInfo::ServiceProvider, QStringLiteral( "openhantek.org" ) );

    QBluetoothServiceInfo::Sequence protocolDescriptorList;
    QBluetoothServiceInfo::Sequence protocol;
    protocol << QVariant::fromValue( QBluetoothUuid( QBluetoothUuid::ProtocolUuid::L2cap ) );
    protocolDescriptorList.append( QVariant::fromValue( protocol ) );
    protocol.clear();
    protocol << QVariant::fromValue( QBluetoothUuid( QBluetoothUuid::ProtocolUuid::Rfcomm ) )
             << QVariant::fromValue( quint8( rfcommServer->serverPort() ) );
    protocolDescriptorList.append( QVariant::fromValue( protocol ) );
    serviceInfo.setAttribute( QBluetoothServiceInfo::ProtocolDescriptorList, protocolDescriptorList );

    if ( !serviceInfo.registerService() ) {
        qCritical() << "BluetoothServer: service registration failed";
        rfcommServer->close();
        delete rfcommServer;
        rfcommServer = nullptr;
        return false;
    }

    connectTelemetry();

    qInfo() << "BluetoothServer: service registered, uuid" << serviceUuid().toString()
            << "- waiting for a client connection";
    return true;
}

void BluetoothServer::stop() {
    if ( serviceInfo.isRegistered() )
        serviceInfo.unregisterService();
    if ( clientSocket ) {
        clientSocket->close();
        clientSocket->deleteLater();
        clientSocket = nullptr;
    }
    if ( rfcommServer ) {
        rfcommServer->close();
        rfcommServer->deleteLater();
        rfcommServer = nullptr;
    }
}

// ---------------------------------------------------------------------------
// telemetry: logic -> client
// ---------------------------------------------------------------------------

void BluetoothServer::connectTelemetry() {
    if ( !dsoControl )
        return;

    connect( dsoControl, &HantekDsoControl::showSamplingStatus, this, [ this ]( bool enabled ) {
        proto::ServerMessage msg;
        msg.mutable_telemetry()->mutable_show_sampling_status()->set_enabled( enabled );
        sendMessage( msg );
    } );
    connect( dsoControl, &HantekDsoControl::statusMessage, this, [ this ]( const QString &text, int timeout ) {
        proto::ServerMessage msg;
        auto *sm = msg.mutable_telemetry()->mutable_status_message();
        sm->set_message( text.toStdString() );
        sm->set_timeout( timeout );
        sendMessage( msg );
    } );
    connect( dsoControl, &HantekDsoControl::samplerateLimitsChanged, this, [ this ]( double minimum, double maximum ) {
        proto::ServerMessage msg;
        auto *sl = msg.mutable_telemetry()->mutable_samplerate_limits_changed();
        sl->set_minimum( minimum );
        sl->set_maximum( maximum );
        sendMessage( msg );
    } );
    connect( dsoControl, &HantekDsoControl::samplerateSet, this, [ this ]( int mode, QList< double > sampleSteps ) {
        proto::ServerMessage msg;
        auto *ss = msg.mutable_telemetry()->mutable_samplerate_set();
        ss->set_mode( mode );
        for ( double step : sampleSteps )
            ss->add_sample_steps( step );
        sendMessage( msg );
    } );
    connect( dsoControl, &HantekDsoControl::samplerateCalculated, this, [ this ]( double samplerate, unsigned oversampling ) {
        proto::ServerMessage msg;
        auto *sc = msg.mutable_telemetry()->mutable_samplerate_calculated();
        sc->set_samplerate( samplerate );
        sc->set_oversampling( oversampling );
        sendMessage( msg );
    } );
    connect( dsoControl, &HantekDsoControl::communicationError, this, [ this ]() {
        proto::ServerMessage msg;
        msg.mutable_telemetry()->mutable_communication_error();
        sendMessage( msg );
    } );
    connect( dsoControl, &HantekDsoControl::deviceAvailabilityChanged, this, [ this ]( bool available ) {
        proto::ServerMessage msg;
        msg.mutable_telemetry()->mutable_device_availability_changed()->set_available( available );
        sendMessage( msg );
    } );
    connect( dsoControl, &HantekDsoControl::liveCalibrationError, this, [ this ]() {
        proto::ServerMessage msg;
        msg.mutable_telemetry()->mutable_live_calibration_error();
        sendMessage( msg );
    } );
    // Direct connection: serialize the raw samples under their read lock in the
    // emitting (DSO) thread; writeFramed() then marshals the bytes to this thread.
    connect( dsoControl, &HantekDsoControl::samplesAvailable, this, &BluetoothServer::onSamplesAvailable,
             Qt::DirectConnection );
}

void BluetoothServer::onSamplesAvailable( const DSOsamples *samples ) {
    if ( !samples )
        return;
    proto::ServerMessage msg;
    auto *out = msg.mutable_telemetry()->mutable_samples();
    {
        QReadLocker locker( &samples->lock );
        out->set_samplerate( samples->samplerate );
        out->set_clipped( samples->clipped );
        out->set_live_trigger( samples->liveTrigger );
        out->set_triggered_position( samples->triggeredPosition );
        out->set_pulse_width1( samples->pulseWidth1 );
        out->set_pulse_width2( samples->pulseWidth2 );
        out->set_free_running( samples->freeRunning );
        out->set_tag( samples->tag );
        for ( const std::vector< double > &channel : samples->data ) {
            auto *ch = out->add_channels();
            for ( double value : channel )
                ch->add_values( value );
        }
    }
    sendMessage( msg );
}

static void fillSampleValues( proto::SampleValues *dst, const SampleValues &src ) {
    dst->set_interval( src.interval );
    for ( double v : src.samples )
        dst->add_samples( v );
}

void BluetoothServer::onProcessingFinished( std::shared_ptr< PPresult > result ) {
    if ( !result )
        return;
    proto::ServerMessage msg;
    auto *out = msg.mutable_telemetry()->mutable_processed_result();
    out->set_software_trigger_triggered( result->softwareTriggerTriggered );
    out->set_triggered_position( result->triggeredPosition );
    out->set_pulse_width1( result->pulseWidth1 );
    out->set_pulse_width2( result->pulseWidth2 );
    out->set_tag( result->tag );
    for ( unsigned c = 0; c < result->channelCount(); ++c ) {
        const DataChannel *data = result->data( c );
        if ( !data )
            continue;
        auto *ch = out->add_channels();
        fillSampleValues( ch->mutable_voltage(), data->voltage );
        fillSampleValues( ch->mutable_spectrum(), data->spectrum );
        ch->set_valid( data->valid );
        ch->set_vmin( data->vmin );
        ch->set_vmax( data->vmax );
        ch->set_rms( data->rms );
        ch->set_db_min( data->dBmin );
        ch->set_db_max( data->dBmax );
        ch->set_dc( data->dc );
        ch->set_ac( data->ac );
        ch->set_db( data->dB );
        ch->set_frequency( data->frequency );
        ch->set_note( data->note.toStdString() );
        ch->set_thd( data->thd );
        ch->set_pulse_width1( data->pulseWidth1 );
        ch->set_pulse_width2( data->pulseWidth2 );
    }
    sendMessage( msg );
}

// ---------------------------------------------------------------------------
// commands: client -> logic
// ---------------------------------------------------------------------------

void BluetoothServer::handleCommand( const proto::Command &command ) {
    if ( !dsoControl )
        return;

    if ( verboseLevel > 2 )
        qDebug() << "BluetoothServer: command" << commandName( command.payload_case() ) << "id" << command.command_id();

    Dso::ErrorCode result = Dso::ErrorCode::NONE;

    // Dispatch onto the DSO control thread and (for slots returning ErrorCode)
    // capture the return value with a blocking queued invocation. This is safe
    // because the server and the control object live in different threads.
    auto invokeRet = [ this, &result ]( auto &&fn ) {
        QMetaObject::invokeMethod( dsoControl, std::forward< decltype( fn ) >( fn ), Qt::BlockingQueuedConnection, &result );
    };
    auto invokeVoid = [ this ]( auto &&fn ) {
        QMetaObject::invokeMethod( dsoControl, std::forward< decltype( fn ) >( fn ), Qt::BlockingQueuedConnection );
    };

    HantekDsoControl *dc = dsoControl;
    switch ( command.payload_case() ) {
    case proto::Command::kEnableSampling: {
        bool enabled = command.enable_sampling().enabled();
        invokeVoid( [ dc, enabled ]() { dc->enableSamplingUI( enabled ); } );
        break;
    }
    case proto::Command::kSetSamplerate: {
        double v = command.set_samplerate().samplerate();
        invokeRet( [ dc, v ]() { return dc->setSamplerate( v ); } );
        break;
    }
    case proto::Command::kSetRecordTime: {
        double v = command.set_record_time().duration();
        invokeRet( [ dc, v ]() { return dc->setRecordTime( v ); } );
        break;
    }
    case proto::Command::kSetChannelUsed: {
        ChannelID ch = command.set_channel_used().channel();
        bool used = command.set_channel_used().used();
        invokeRet( [ dc, ch, used ]() { return dc->setChannelUsed( ch, used ); } );
        break;
    }
    case proto::Command::kSetChannelInverted: {
        ChannelID ch = command.set_channel_inverted().channel();
        bool inv = command.set_channel_inverted().inverted();
        invokeRet( [ dc, ch, inv ]() { return dc->setChannelInverted( ch, inv ); } );
        break;
    }
    case proto::Command::kSetProbe: {
        ChannelID ch = command.set_probe().channel();
        double attn = command.set_probe().probe_attn();
        invokeRet( [ dc, ch, attn ]() { return dc->setProbe( ch, attn ); } );
        break;
    }
    case proto::Command::kSetGain: {
        ChannelID ch = command.set_gain().channel();
        double gain = command.set_gain().gain();
        invokeRet( [ dc, ch, gain ]() { return dc->setGain( ch, gain ); } );
        break;
    }
    case proto::Command::kSetCoupling: {
        ChannelID ch = command.set_coupling().channel();
        Dso::Coupling coupling = toDso( command.set_coupling().coupling() );
        invokeRet( [ dc, ch, coupling ]() { return dc->setCoupling( ch, coupling ); } );
        break;
    }
    case proto::Command::kSetTriggerMode: {
        Dso::TriggerMode mode = toDso( command.set_trigger_mode().mode() );
        invokeRet( [ dc, mode ]() { return dc->setTriggerMode( mode ); } );
        break;
    }
    case proto::Command::kSetTriggerSource: {
        int channel = command.set_trigger_source().channel();
        invokeRet( [ dc, channel ]() { return dc->setTriggerSource( channel ); } );
        break;
    }
    case proto::Command::kSetTriggerSmooth: {
        int smooth = command.set_trigger_smooth().smooth();
        invokeRet( [ dc, smooth ]() { return dc->setTriggerSmooth( smooth ); } );
        break;
    }
    case proto::Command::kSetTriggerLevel: {
        ChannelID ch = command.set_trigger_level().channel();
        double level = command.set_trigger_level().level();
        invokeRet( [ dc, ch, level ]() { return dc->setTriggerLevel( ch, level ); } );
        break;
    }
    case proto::Command::kSetTriggerSlope: {
        Dso::Slope slope = toDso( command.set_trigger_slope().slope() );
        invokeRet( [ dc, slope ]() { return dc->setTriggerSlope( slope ); } );
        break;
    }
    case proto::Command::kSetTriggerPosition: {
        double pos = command.set_trigger_position().position();
        invokeRet( [ dc, pos ]() { return dc->setTriggerPosition( pos ); } );
        break;
    }
    case proto::Command::kSetCalFreq: {
        double v = command.set_cal_freq().calfreq();
        invokeRet( [ dc, v ]() { return dc->setCalFreq( v ); } );
        break;
    }
    case proto::Command::kRestartSampling: {
        invokeVoid( [ dc ]() { dc->restartSampling(); } );
        break;
    }
    case proto::Command::kCalibrateOffset: {
        bool enable = command.calibrate_offset().enable();
        invokeVoid( [ dc, enable ]() { dc->calibrateOffset( enable ); } );
        break;
    }
    case proto::Command::kStringCommand: {
        QString s = QString::fromStdString( command.string_command().command() );
        invokeRet( [ dc, s ]() { return dc->stringCommand( s ); } );
        break;
    }
    case proto::Command::PAYLOAD_NOT_SET:
    default:
        result = Dso::ErrorCode::PARAMETER;
        break;
    }

    // Acknowledge the command.
    proto::ServerMessage ackMsg;
    auto *ack = ackMsg.mutable_ack();
    ack->set_command_id( command.command_id() );
    ack->set_error_code( toProto( result ) );
    if ( verboseLevel > 2 )
        qDebug() << "BluetoothServer: ack id" << command.command_id() << "errorCode" << int( result );
    sendMessage( ackMsg );
}

// ---------------------------------------------------------------------------
// connection handling
// ---------------------------------------------------------------------------

void BluetoothServer::onNewConnection() {
    QBluetoothSocket *socket = rfcommServer->nextPendingConnection();
    if ( !socket )
        return;
    // Single active client: replace any previous connection.
    if ( clientSocket ) {
        clientSocket->close();
        clientSocket->deleteLater();
    }
    rxBuffer.clear();
    clientSocket = socket;
    connect( clientSocket, &QBluetoothSocket::readyRead, this, &BluetoothServer::onReadyRead );
    connect( clientSocket, &QBluetoothSocket::disconnected, this, &BluetoothServer::onClientDisconnected );
    qInfo() << "BluetoothServer: client connected -" << clientSocket->peerName() << clientSocket->peerAddress().toString();
}

void BluetoothServer::onClientDisconnected() {
    qInfo() << "BluetoothServer: client disconnected";
    if ( clientSocket ) {
        clientSocket->deleteLater();
        clientSocket = nullptr;
    }
    rxBuffer.clear();
}

void BluetoothServer::onReadyRead() {
    if ( !clientSocket )
        return;
    rxBuffer.append( clientSocket->readAll() );
    if ( verboseLevel > 4 )
        qDebug() << "BluetoothServer: rx buffer now" << rxBuffer.size() << "bytes";

    // Decode as many complete [uint32 length][payload] frames as available.
    for ( ;; ) {
        if ( rxBuffer.size() < 4 )
            return;
        const uchar *p = reinterpret_cast< const uchar * >( rxBuffer.constData() );
        quint32 length = ( quint32( p[ 0 ] ) << 24 ) | ( quint32( p[ 1 ] ) << 16 ) | ( quint32( p[ 2 ] ) << 8 ) | quint32( p[ 3 ] );
        if ( quint32( rxBuffer.size() ) < 4 + length )
            return; // wait for the rest of the frame
        QByteArray payload = rxBuffer.mid( 4, int( length ) );
        rxBuffer.remove( 0, int( 4 + length ) );
        if ( verboseLevel > 3 )
            qDebug() << "BluetoothServer: received frame," << length << "bytes";

        proto::ClientMessage message;
        if ( !message.ParseFromArray( payload.constData(), payload.size() ) ) {
            qWarning() << "BluetoothServer: failed to parse client message (" << length << "bytes )";
            continue;
        }
        if ( message.message_case() == proto::ClientMessage::kCommand )
            handleCommand( message.command() );
        else
            qWarning() << "BluetoothServer: received message with no command payload";
    }
}

// ---------------------------------------------------------------------------
// transport
// ---------------------------------------------------------------------------

void BluetoothServer::sendMessage( const proto::ServerMessage &message ) {
    if ( verboseLevel > 4 && message.message_case() == proto::ServerMessage::kTelemetry )
        qDebug() << "BluetoothServer: send telemetry" << telemetryName( message.telemetry().payload_case() );
    std::string bytes;
    if ( !message.SerializeToString( &bytes ) ) {
        qWarning() << "BluetoothServer: failed to serialize server message";
        return;
    }
    writeFramed( QByteArray( bytes.data(), int( bytes.size() ) ) );
}

void BluetoothServer::writeFramed( const QByteArray &payload ) {
    // Always write from the thread that owns the socket.
    if ( QThread::currentThread() != this->thread() ) {
        QMetaObject::invokeMethod( this, [ this, payload ]() { writeFramed( payload ); }, Qt::QueuedConnection );
        return;
    }
    if ( !clientSocket || clientSocket->state() != QBluetoothSocket::SocketState::ConnectedState )
        return;
    quint32 len = quint32( payload.size() );
    char header[ 4 ] = { char( ( len >> 24 ) & 0xFF ), char( ( len >> 16 ) & 0xFF ), char( ( len >> 8 ) & 0xFF ),
                         char( len & 0xFF ) };
    clientSocket->write( header, 4 );
    clientSocket->write( payload );
}










