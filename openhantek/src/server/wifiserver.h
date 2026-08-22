// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <QByteArray>
#include <QHash>
#include <QList>
#include <QObject>
#include <QtGlobal>
#include <memory>

class QTcpServer;
class QTcpSocket;

class HantekDsoControl;
class PPresult;
struct DSOsamples;

namespace openhantek {
namespace proto {
class Command;
class ServerMessage;
} // namespace proto
} // namespace openhantek

/// \brief Headless WiFi (TCP/IP) bridge for the OpenHantek logic layer.
///
/// This is the WiFi counterpart of BluetoothServer: instead of wiring the
/// docks/widgets to the HantekDsoControl slots and signals, it:
///   * decodes protobuf Command frames received over a TCP socket and
///     dispatches them to the matching HantekDsoControl slot (thread-safe,
///     honouring the control object's thread affinity), and
///   * serializes the HantekDsoControl signals plus the post-processing result
///     into protobuf Telemetry frames streamed back to all connected clients.
///
/// Framing on the stream: [uint32 big-endian length][protobuf payload].
///
/// Unlike the single-client Bluetooth RFCOMM bridge, WifiServer accepts
/// multiple simultaneous TCP clients on the local network: telemetry is
/// broadcast to every connected client, while command acknowledgements are
/// sent back only to the client that issued the command.
///
/// The object must live in the thread that owns the QTcpServer/sockets (the
/// main QCoreApplication thread). Signal handlers may be invoked from the
/// DSO/post processing threads; outgoing writes are always marshalled to this
/// thread.
class WifiServer : public QObject {
    Q_OBJECT

  public:
    /// Default TCP port the server listens on.
    static constexpr quint16 DEFAULT_PORT = 6022;

    /// \param dsoControl The (thread-affine) logic controller to drive. Not owned.
    /// \param port TCP port to listen on (all interfaces).
    /// \param verboseLevel Tracing verbosity, mirrors the global program option.
    /// \param parent Qt parent.
    explicit WifiServer( HantekDsoControl *dsoControl, quint16 port = DEFAULT_PORT, int verboseLevel = 0,
                         QObject *parent = nullptr );
    ~WifiServer() override;

    /// \brief Start listening on all interfaces.
    /// \return true on success.
    bool start();

    /// \brief Stop the server, closing all client connections.
    void stop();

    /// The port actually bound (useful when 0 was requested for an ephemeral port).
    quint16 serverPort() const;

  public slots:
    /// Connected to PostProcessing::processingFinished to stream analysis results.
    void onProcessingFinished( std::shared_ptr< PPresult > result );

  private slots:
    void onNewConnection();
    void onClientDisconnected();
    void onReadyRead();

  private:
    // --- command handling (client -> logic) ---
    void handleCommand( QTcpSocket *origin, const openhantek::proto::Command &command );

    // --- telemetry handling (logic -> client) ---
    void connectTelemetry();
    void onSamplesAvailable( const DSOsamples *samples );

    // --- transport ---
    void sendTo( QTcpSocket *socket, const openhantek::proto::ServerMessage &message ); ///< reply to one client
    void broadcast( const openhantek::proto::ServerMessage &message );                  ///< telemetry to all clients
    void writeFramed( QTcpSocket *socket, const QByteArray &payload );                  ///< thread-safe framed write

    HantekDsoControl *dsoControl = nullptr;
    quint16 port = DEFAULT_PORT;
    int verboseLevel = 0;
    QTcpServer *tcpServer = nullptr;
    QList< QTcpSocket * > clients;
    QHash< QTcpSocket *, QByteArray > rxBuffers; ///< accumulates partial frames per client
};

