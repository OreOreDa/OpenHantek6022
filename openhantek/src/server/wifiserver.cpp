// SPDX-License-Identifier: GPL-2.0-or-later

#include "wifiserver.h"

#include <QDebug>
#include <QHostAddress>
#include <QReadLocker>
#include <QTcpServer>
#include <QTcpSocket>
#include <QThread>

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

WifiServer::WifiServer( HantekDsoControl *dsoControl, quint16 port, int verboseLevel, QObject *parent )
    : QObject( parent ), dsoControl( dsoControl ), port( port ), verboseLevel( verboseLevel ) {}

WifiServer::~WifiServer() { stop(); }

bool WifiServer::start() {
    if ( tcpServer )
        return true;

    qInfo() << "WifiServer: starting TCP server on port" << port << "...";
    tcpServer = new QTcpServer( this );
    connect( tcpServer, &QTcpServer::newConnection, this, &WifiServer::onNewConnection );

    // Bind to all interfaces so clients on the same WiFi/LAN network can connect.
    if ( !tcpServer->listen( QHostAddress::Any, port ) ) {
        qCritical() << "WifiServer: cannot listen on port" << port << "-" << tcpServer->errorString();
        delete tcpServer;
        tcpServer = nullptr;
        return false;
    }
    qInfo() << "WifiServer: listening on TCP port" << tcpServer->serverPort() << "(all interfaces)";

    connectTelemetry();

    qInfo() << "WifiServer: waiting for client connections";
    return true;
}

void WifiServer::stop() {
    for ( QTcpSocket *socket : clients ) {
        socket->close();
        socket->deleteLater();
    }
    clients.clear();
    rxBuffers.clear();
    if ( tcpServer ) {
        tcpServer->close();
        tcpServer->deleteLater();
        tcpServer = nullptr;
    }
}

quint16 WifiServer::serverPort() const { return tcpServer ? tcpServer->serverPort() : port; }

// ---------------------------------------------------------------------------
// telemetry: logic -> client(s)
// ---------------------------------------------------------------------------

void WifiServer::connectTelemetry() {
    if ( !dsoControl )
        return;

    connect( dsoControl, &HantekDsoControl::showSamplingStatus, this, [ this ]( bool enabled ) {
        proto::ServerMessage msg;
        msg.mutable_telemetry()->mutable_show_sampling_status()->set_enabled( enabled );
        broadcast( msg );
    } );
    connect( dsoControl, &HantekDsoControl::statusMessage, this, [ this ]( const QString &text, int timeout ) {
        proto::ServerMessage msg;
        auto *sm = msg.mutable_telemetry()->mutable_status_message();
        sm->set_message( text.toStdString() );
        sm->set_timeout( timeout );
        broadcast( msg );
    } );
    connect( dsoControl, &HantekDsoControl::samplerateLimitsChanged, this, [ this ]( double minimum, double maximum ) {
        proto::ServerMessage msg;
        auto *sl = msg.mutable_telemetry()->mutable_samplerate_limits_changed();
        sl->set_minimum( minimum );
        sl->set_maximum( maximum );
        broadcast( msg );
    } );
    connect( dsoControl, &HantekDsoControl::samplerateSet, this, [ this ]( int mode, QList< double > sampleSteps ) {
        proto::ServerMessage msg;
        auto *ss = msg.mutable_telemetry()->mutable_samplerate_set();
        ss->set_mode( mode );
        for ( double step : sampleSteps )
            ss->add_sample_steps( step );
        broadcast( msg );
    } );
    connect( dsoControl, &HantekDsoControl::samplerateCalculated, this, [ this ]( double samplerate, unsigned oversampling ) {
        proto::ServerMessage msg;
        auto *sc = msg.mutable_telemetry()->mutable_samplerate_calculated();
        sc->set_samplerate( samplerate );
        sc->set_oversampling( oversampling );
        broadcast( msg );
    } );
    connect( dsoControl, &HantekDsoControl::communicationError, this, [ this ]() {
        proto::ServerMessage msg;
        msg.mutable_telemetry()->mutable_communication_error();
        broadcast( msg );
    } );
    connect( dsoControl, &HantekDsoControl::deviceAvailabilityChanged, this, [ this ]( bool available ) {
        proto::ServerMessage msg;
        msg.mutable_telemetry()->mutable_device_availability_changed()->set_available( available );
        broadcast( msg );
    } );
    connect( dsoControl, &HantekDsoControl::liveCalibrationError, this, [ this ]() {
        proto::ServerMessage msg;
        msg.mutable_telemetry()->mutable_live_calibration_error();
        broadcast( msg );
    } );
    // Direct connection: serialize the raw samples under their read lock in the
    // emitting (DSO) thread; writeFramed() then marshals the bytes to this thread.
    connect( dsoControl, &HantekDsoControl::samplesAvailable, this, &WifiServer::onSamplesAvailable, Qt::DirectConnection );
}

void WifiServer::onSamplesAvailable( const DSOsamples *samples ) {
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
    broadcast( msg );
}

static void fillSampleValues( proto::SampleValues *dst, const SampleValues &src ) {
    dst->set_interval( src.interval );
    for ( double v : src.samples )
        dst->add_samples( v );
}

void WifiServer::onProcessingFinished( std::shared_ptr< PPresult > result ) {
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
    broadcast( msg );
}

// ---------------------------------------------------------------------------
// commands: client -> logic
// ---------------------------------------------------------------------------

void WifiServer::handleCommand( QTcpSocket *origin, const proto::Command &command ) {
    if ( !dsoControl )
        return;

    if ( verboseLevel > 2 )
        qDebug() << "WifiServer: command" << commandName( command.payload_case() ) << "id" << command.command_id();

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

    // Acknowledge the command back to the client that issued it.
    proto::ServerMessage ackMsg;
    auto *ack = ackMsg.mutable_ack();
    ack->set_command_id( command.command_id() );
    ack->set_error_code( toProto( result ) );
    if ( verboseLevel > 2 )
        qDebug() << "WifiServer: ack id" << command.command_id() << "errorCode" << int( result );
    sendTo( origin, ackMsg );
}

// ---------------------------------------------------------------------------
// connection handling
// ---------------------------------------------------------------------------

void WifiServer::onNewConnection() {
    while ( tcpServer->hasPendingConnections() ) {
        QTcpSocket *socket = tcpServer->nextPendingConnection();
        if ( !socket )
            continue;
        clients.append( socket );
        rxBuffers[ socket ] = QByteArray();
        connect( socket, &QTcpSocket::readyRead, this, &WifiServer::onReadyRead );
        connect( socket, &QTcpSocket::disconnected, this, &WifiServer::onClientDisconnected );
        qInfo() << "WifiServer: client connected -" << socket->peerAddress().toString() << ":" << socket->peerPort()
                << "(" << clients.size() << "active )";
    }
}

void WifiServer::onClientDisconnected() {
    auto *socket = qobject_cast< QTcpSocket * >( sender() );
    if ( !socket )
        return;
    qInfo() << "WifiServer: client disconnected -" << socket->peerAddress().toString() << ":" << socket->peerPort();
    clients.removeAll( socket );
    rxBuffers.remove( socket );
    socket->deleteLater();
}

void WifiServer::onReadyRead() {
    auto *socket = qobject_cast< QTcpSocket * >( sender() );
    if ( !socket || !rxBuffers.contains( socket ) )
        return;
    QByteArray &rxBuffer = rxBuffers[ socket ];
    rxBuffer.append( socket->readAll() );
    if ( verboseLevel > 4 )
        qDebug() << "WifiServer: rx buffer now" << rxBuffer.size() << "bytes";

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
            qDebug() << "WifiServer: received frame," << length << "bytes";

        proto::ClientMessage message;
        if ( !message.ParseFromArray( payload.constData(), payload.size() ) ) {
            qWarning() << "WifiServer: failed to parse client message (" << length << "bytes )";
            continue;
        }
        if ( message.message_case() == proto::ClientMessage::kCommand )
            handleCommand( socket, message.command() );
        else
            qWarning() << "WifiServer: received message with no command payload";
    }
}

// ---------------------------------------------------------------------------
// transport
// ---------------------------------------------------------------------------

void WifiServer::sendTo( QTcpSocket *socket, const proto::ServerMessage &message ) {
    if ( !socket )
        return;
    std::string bytes;
    if ( !message.SerializeToString( &bytes ) ) {
        qWarning() << "WifiServer: failed to serialize server message";
        return;
    }
    writeFramed( socket, QByteArray( bytes.data(), int( bytes.size() ) ) );
}

void WifiServer::broadcast( const proto::ServerMessage &message ) {
    if ( verboseLevel > 4 && message.message_case() == proto::ServerMessage::kTelemetry )
        qDebug() << "WifiServer: broadcast telemetry" << telemetryName( message.telemetry().payload_case() ) << "to"
                 << clients.size() << "client(s)";
    std::string bytes;
    if ( !message.SerializeToString( &bytes ) ) {
        qWarning() << "WifiServer: failed to serialize server message";
        return;
    }
    QByteArray payload( bytes.data(), int( bytes.size() ) );
    // Marshal the client list access to this object's thread, then fan out.
    if ( QThread::currentThread() != this->thread() ) {
        QMetaObject::invokeMethod( this, [ this, payload ]() {
            for ( QTcpSocket *socket : clients )
                writeFramed( socket, payload );
        }, Qt::QueuedConnection );
        return;
    }
    for ( QTcpSocket *socket : clients )
        writeFramed( socket, payload );
}

void WifiServer::writeFramed( QTcpSocket *socket, const QByteArray &payload ) {
    // Always write from the thread that owns the socket.
    if ( QThread::currentThread() != this->thread() ) {
        QMetaObject::invokeMethod( this, [ this, socket, payload ]() { writeFramed( socket, payload ); }, Qt::QueuedConnection );
        return;
    }
    if ( !socket || socket->state() != QAbstractSocket::ConnectedState )
        return;
    quint32 len = quint32( payload.size() );
    char header[ 4 ] = { char( ( len >> 24 ) & 0xFF ), char( ( len >> 16 ) & 0xFF ), char( ( len >> 8 ) & 0xFF ),
                         char( len & 0xFF ) };
    socket->write( header, 4 );
    socket->write( payload );
}

