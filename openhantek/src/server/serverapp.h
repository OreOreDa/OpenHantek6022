// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/// \brief Entry point for the headless (no GUI / no X server) OpenHantek6022 server.
///
/// Builds the same logic layer as the GUI application (device, HantekDsoControl,
/// post processing, exporters, capturing) but drives it through a Bluetooth
/// RFCOMM protobuf server instead of the Qt widgets. Uses QCoreApplication, so
/// it pulls in no QtWidgets/OpenGL dependency and runs without a display.
///
/// \param argc, argv Forwarded from main().
/// \return Process exit code.
int runHeadlessServer( int argc, char **argv );

