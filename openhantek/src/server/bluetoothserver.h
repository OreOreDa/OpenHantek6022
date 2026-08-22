// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <QByteArray>
#include <QList>
#include <QObject>
#include <QtGlobal>
#include <memory>

#include <QtBluetooth/QBluetoothServiceInfo>
#include <QtBluetooth/QBluetoothUuid>

class QBluetoothServer;
class QBluetoothSocket;

class HantekDsoControl;
class PPresult;
struct DSOsamples;

namespace openhantek {
namespace proto {
class Command;
class ServerMessage;
} // namespace proto
} // namespace openhantek

/// \brief Headless Bluetooth RFCOMM bridge for the OpenHantek logic layer.
///
/// This class is the server-side counterpart of the GUI. Instead of wiring the
/// docks/widgets to the HantekDsoControl slots and signals, it:
///   * decodes protobuf Command frames received over an RFCOMM socket and
///     dispatches them to the matching HantekDsoControl slot (thread-safe,
///     honouring the control object's thread affinity), and
///   * serializes the HantekDsoControl signals plus the post-processing result
///     into protobuf Telemetry frames streamed back to the client.
///
/// Framing on the stream: [uint32 big-endian length][protobuf payload].
///
/// The object must live in the thread that owns the RFCOMM socket (the main
/// QCoreApplication thread). Signal handlers may be invoked from the DSO/post
/// processing threads; outgoing writes are always marshalled to this thread.
class BluetoothServer : public QObject {
    Q_OBJECT

  public:
    /// Default RFCOMM service UUID advertised by the server.
    static const QBluetoothUuid serviceUuid();

    /// \param dsoControl The (thread-affine) logic controller to drive. Not owned.
    /// \param verboseLevel Tracing verbosity, mirrors the global program option.
    /// \param parent Qt parent.
    explicit BluetoothServer( HantekDsoControl *dsoControl, int verboseLevel = 0, QObject *parent = nullptr );
    ~BluetoothServer() override;

    /// \brief Start listening and register the RFCOMM service record.
    /// \return true on success.
    bool start();

    /// \brief Stop the server and unregister the service.
    void stop();

  public slots:
    /// Connected to PostProcessing::processingFinished to stream analysis results.
    void onProcessingFinished( std::shared_ptr< PPresult > result );

  private slots:
    void onNewConnection();
    void onClientDisconnected();
    void onReadyRead();

  private:
    // --- command handling (client -> logic) ---
    void handleCommand( const openhantek::proto::Command &command );

    // --- telemetry handling (logic -> client) ---
    void connectTelemetry();
    void onSamplesAvailable( const DSOsamples *samples );

    // --- transport ---
    void sendMessage( const openhantek::proto::ServerMessage &message );
    void writeFramed( const QByteArray &payload ); ///< thread-safe framed write

    HantekDsoControl *dsoControl = nullptr;
    int verboseLevel = 0;
    QBluetoothServer *rfcommServer = nullptr;
    QBluetoothServiceInfo serviceInfo;
    QBluetoothSocket *clientSocket = nullptr;
    QByteArray rxBuffer; ///< accumulates partial frames from the client
};



