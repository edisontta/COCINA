#!/usr/bin/env python3
"""
k902_nmea_server.py
--------------------

This script provides a minimal example of how to use a SinoGNSS K902 GNSS module
connected to an Orange Pi via a UART to output standard NMEA sentences and
re-distribute them over a TCP socket.  It performs three key tasks:

1. Opens a serial connection to the K902 (COM1) through the Orange Pi UART.
2. Sends a set of initialization commands to configure the module for standard
   NMEA output (GPGGA, GPGSV, GPRMC, GPVTG, etc.) and saves the
   configuration.
3. Runs a simple multi‑client TCP server; connected clients will receive
   streamed NMEA sentences as they arrive.

This allows you to power the K902 from the Orange Pi, configure it once on
startup, and then point mobile surveying software (e.g. Survey Master,
FieldGenius, QGIS) at the Orange Pi’s IP address and TCP port to consume
live positioning data.

Usage example:

    python3 k902_nmea_server.py --serial /dev/ttyS3 --baud 115200 --port 5001

Before running this script, ensure that:

  * The K902 is powered (VCC+5V and GND connected) and its COM1 TX/RX are
    wired to the Orange Pi’s RX/TX pins (or via a USB‑TTL adapter).
  * pyserial is installed (`sudo apt install python3-serial`).
  * Your firewall allows incoming connections on the chosen TCP port.

This script is provided as a basic reference and may need to be adapted for
production use (e.g. adding error handling, authentication, NTRIP formatting,
etc.).

Author: ChatGPT
License: MIT
"""

import argparse
import serial
import socket
import threading
import time
from typing import List


def send_initialization_commands(ser: serial.Serial, commands: List[bytes], delay: float = 0.2) -> None:
    """Send a series of initialization commands to the GNSS module.

    Each command in ``commands`` should be a ``bytes`` object ending with
    ``\r\n``.  A small delay is inserted between writes to give the module
    time to process the command.

    :param ser: An open ``serial.Serial`` instance.
    :param commands: List of commands to send.
    :param delay: Delay in seconds between commands.
    """
    for cmd in commands:
        ser.write(cmd)
        ser.flush()
        time.sleep(delay)


class NMEATcpServer:
    """A simple multi‑client TCP server that broadcasts NMEA lines.

    The server listens on ``host`` and ``port`` and relays lines read from
    ``ser`` to all connected clients.  It handles clients in a single
    broadcasting thread; if a client disconnects, its socket is removed.
    """

    def __init__(self, ser: serial.Serial, host: str = "0.0.0.0", port: int = 5001) -> None:
        self.ser = ser
        self.host = host
        self.port = port
        self.clients: List[socket.socket] = []
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._stop_event = threading.Event()

    def start(self) -> None:
        """Start the TCP server and the serial‑read thread."""
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(5)
        print(f"NMEA TCP server listening on {self.host}:{self.port}")
        threading.Thread(target=self._accept_loop, daemon=True).start()
        threading.Thread(target=self._broadcast_loop, daemon=True).start()

    def stop(self) -> None:
        """Stop the server and close all sockets."""
        self._stop_event.set()
        for client in self.clients:
            try:
                client.shutdown(socket.SHUT_RDWR)
                client.close()
            except Exception:
                pass
        try:
            self.server_sock.close()
        except Exception:
            pass

    def _accept_loop(self) -> None:
        """Accept incoming client connections until stopped."""
        while not self._stop_event.is_set():
            try:
                client_sock, addr = self.server_sock.accept()
                print(f"Client connected from {addr}")
                client_sock.setblocking(False)
                self.clients.append(client_sock)
            except Exception:
                time.sleep(0.1)

    def _broadcast_loop(self) -> None:
        """Read lines from the serial port and broadcast to all clients."""
        while not self._stop_event.is_set():
            try:
                line = self.ser.readline()
                if not line:
                    continue
                # Broadcast to all connected clients
                disconnected = []
                for client in self.clients:
                    try:
                        client.sendall(line)
                    except Exception:
                        disconnected.append(client)
                # Remove disconnected clients
                for client in disconnected:
                    print("Removing disconnected client")
                    try:
                        client.close()
                    except Exception:
                        pass
                    self.clients.remove(client)
            except Exception:
                time.sleep(0.1)


def main() -> None:
    parser = argparse.ArgumentParser(description="K902 NMEA server for Orange Pi")
    parser.add_argument("--serial", default="/dev/ttyS3", help="Serial device connected to K902 (default: /dev/ttyS3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate for serial connection (default: 115200)")
    parser.add_argument("--port", type=int, default=5001, help="TCP port to listen on for NMEA clients (default: 5001)")
    args = parser.parse_args()

    # Open serial port
    try:
        ser = serial.Serial(args.serial, args.baud, timeout=1)
    except Exception as e:
        print(f"Error opening serial port {args.serial}: {e}")
        return

    # Initialisation commands for K902
    init_cmds = [
        b"set nmeamsgformat standard\r\n",
        b"interfacemode com1 auto auto on\r\n",
        b"unlogall com1\r\n",
        b"log com1 gpgga ontime 1\r\n",
        b"log com1 gpgsa ontime 1\r\n",
        b"log com1 gpgsv ontime 1\r\n",
        b"log com1 gprmc ontime 1\r\n",
        b"log com1 gpvtg ontime 1\r\n",
        b"log com1 gpgst ontime 1\r\n",
        b"saveconfig\r\n",
    ]
    print("Sending initialization commands to K902…")
    send_initialization_commands(ser, init_cmds)
    print("Initialization complete. Starting TCP server…")

    # Start TCP server
    server = NMEATcpServer(ser, port=args.port)
    server.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping server…")
        server.stop()
        ser.close()


if __name__ == "__main__":
    main()