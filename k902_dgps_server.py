#!/usr/bin/env python3
"""
k902_dgps_server.py
--------------------

This script combines several of the building blocks contained in the Python
modules you supplied to provide a simple, self‑contained service that turns
an Orange Pi with a connected SinoGNSS K902 module into a small
network‑accessible differential GNSS receiver.  It takes care of three
distinct tasks:

1. **Configure the K902 via UART.**  On start‑up the script opens the
   specified serial port at the specified baud rate, sends a series of
   initialisation commands to the K902 to enable standard NMEA output on
   COM1, and optionally stores the configuration so it persists after
   power‑cycling.
2. **Optionally connect to an NTRIP caster.**  If NTRIP details are
   supplied (host, port, mountpoint, username, password), the script
   establishes a persistent HTTP connection to the caster and streams the
   received correction data back to the K902 over the same serial port.
3. **Serve the NMEA stream over TCP.**  A simple multi‑client TCP server
   listens on a configurable port and relays every NMEA sentence read from
   the serial port to all connected clients.  Surveying apps running on
   your mobile phone (such as Survey Master, FieldGenius, QGIS, etc.) can
   connect to this TCP port over Wi‑Fi and consume real‑time positioning
   data.

In addition, the script can optionally activate a Wi‑Fi hotspot on the
Orange Pi using ``nmcli``.  This is a convenience feature so that a phone
can connect directly to the Orange Pi without an existing wireless network.
The hotspot parameters (SSID and password) are configurable.

**Usage example**
```
sudo python3 k902_dgps_server.py \
    --serial /dev/ttyS3 --baud 115200 \
    --nmea-port 5001 \
    --ntrip-host example.caster.org --ntrip-port 2101 \
    --ntrip-mountpoint MYMOUNT \
    --ntrip-user myuser --ntrip-pass mypassword \
    --hotspot-ssid GNSS_Pi --hotspot-pass 12345678
```

**Prerequisites**

- The K902 must be connected to the Orange Pi via its UART1 (TX/RX) and
  powered appropriately (3.3–5.5 V on the VCC pin).  Ensure that the logic
  levels are compatible (if the K902 outputs 5 V on TXD, use a level
  shifter before connecting to the Orange Pi’s RX pin).
- Python 3 with the ``pyserial`` library installed.  On a Debian‑based
  system you can install it using ``sudo apt install python3‑serial``.
- If you intend to use the Wi‑Fi hotspot feature, ``nmcli`` must be
  available and the network interface name must be ``wlan0``.  You may
  need to adjust the commands if your interface differs.

This script is intentionally lean and does not replicate every feature of
the larger project you provided.  Instead it focuses on the core
functionality required to make the Orange Pi act as a differential GNSS
receiver that can talk to Survey Master or similar applications.  Feel free
to extend it with additional error handling, logging or other features as
needed.

Author: ChatGPT
License: MIT
"""

import argparse
import base64
import os
import socket
import subprocess
import threading
import time
from typing import List, Optional

import serial


def send_initialization_commands(ser: serial.Serial, commands: List[bytes], delay: float = 0.2) -> None:
    """Send a series of commands to the serial port with delays.

    Each command should be a bytes object ending in ``\r\n``.  A small
    delay is inserted after each command to give the K902 time to process it.
    """
    for cmd in commands:
        ser.write(cmd)
        ser.flush()
        time.sleep(delay)


class NMEATcpServer:
    """A simple multi‑client TCP server that broadcasts serial lines.

    When started, this server listens on the specified host and port.  It
    reads lines from the provided ``serial.Serial`` instance and forwards
    them to all connected clients.  If a client disconnects, it is
    removed from the internal list.
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
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(5)
        print(f"NMEA TCP server listening on {self.host}:{self.port}")
        threading.Thread(target=self._accept_loop, daemon=True).start()
        threading.Thread(target=self._broadcast_loop, daemon=True).start()

    def stop(self) -> None:
        self._stop_event.set()
        try:
            self.server_sock.close()
        except Exception:
            pass
        for client in list(self.clients):
            try:
                client.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                client.close()
            except Exception:
                pass
        self.clients.clear()

    def _accept_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                client_sock, addr = self.server_sock.accept()
                print(f"Client connected from {addr}")
                client_sock.setblocking(False)
                self.clients.append(client_sock)
            except Exception:
                time.sleep(0.1)

    def _broadcast_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                line = self.ser.readline()
                if not line:
                    continue
                disconnected: List[socket.socket] = []
                for client in self.clients:
                    try:
                        client.sendall(line)
                    except Exception:
                        disconnected.append(client)
                for client in disconnected:
                    try:
                        client.close()
                    except Exception:
                        pass
                    try:
                        self.clients.remove(client)
                    except Exception:
                        pass
            except Exception:
                time.sleep(0.1)


class NTRIPClient(threading.Thread):
    """A minimal NTRIP client to fetch RTCM corrections and feed them to serial.

    NTRIP (Networked Transport of RTCM via Internet Protocol) is a protocol
    used to deliver GNSS differential corrections from a caster to a rover.
    This client establishes an HTTP connection to the specified caster
    endpoint and continuously streams the received data to the provided
    serial port.  It attempts to reconnect in case of errors.
    """

    def __init__(self, host: str, port: int, mountpoint: str,
                 username: Optional[str], password: Optional[str],
                 ser: serial.Serial, retry_delay: float = 5.0) -> None:
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.mountpoint = mountpoint
        self.username = username
        self.password = password
        self.ser = ser
        self.retry_delay = retry_delay
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        while not self._stop_event.is_set():
            sock = None
            try:
                print(f"Connecting to NTRIP caster {self.host}:{self.port}/{self.mountpoint}")
                sock = socket.create_connection((self.host, self.port), timeout=10)
                # Build HTTP GET request for NTRIP
                request_lines = [
                    f"GET /{self.mountpoint} HTTP/1.0",
                    f"User-Agent: NTRIP k902_dgps_client",
                    f"Accept: */*",
                    f"Connection: close",
                ]
                # Add basic auth header if username is provided
                if self.username:
                    credentials = f"{self.username}:{self.password or ''}"
                    auth_header = base64.b64encode(credentials.encode()).decode()
                    request_lines.append(f"Authorization: Basic {auth_header}")
                request_lines.append("")
                request_lines.append("")
                request_data = "\r\n".join(request_lines).encode()
                sock.sendall(request_data)
                # Skip HTTP headers
                headers_buffer = b""
                while b"\r\n\r\n" not in headers_buffer:
                    headers_buffer += sock.recv(1)
                print("Connected to NTRIP caster; streaming corrections…")
                # Now stream the body to serial
                while not self._stop_event.is_set():
                    data = sock.recv(4096)
                    if not data:
                        break
                    self.ser.write(data)
                print("NTRIP connection ended; reconnecting…")
            except Exception as e:
                print(f"NTRIP connection error: {e}")
            finally:
                try:
                    if sock:
                        sock.close()
                except Exception:
                    pass
            # Wait before retrying
            time.sleep(self.retry_delay)


def activate_hotspot(ssid: str, password: str) -> None:
    """Activate a Wi‑Fi hotspot using nmcli on wlan0.

    This function attempts to create or activate a hotspot with the given
    SSID and password.  It follows the same logic as the original
    ``WifiClient`` class: if a connection named "Hotspot" already exists
    it brings it up; otherwise it creates a new connection.  Errors are
    printed but not considered fatal.
    """
    # Check if the Hotspot connection exists
    try:
        print(f"Activating Wi‑Fi hotspot '{ssid}'…")
        # Look for an existing connection with the given SSID
        result = subprocess.run([
            "nmcli", "-t", "-f", "NAME", "connection", "show"
        ], capture_output=True, text=True)
        existing = False
        if result.returncode == 0:
            names = result.stdout.strip().split("\n")
            existing = ssid in names
        if not existing:
            # Create a new hotspot connection
            cmds = [
                ["nmcli", "con", "add", "type", "wifi", "ifname", "wlan0",
                 "con-name", ssid, "autoconnect", "yes", "ssid", ssid],
                ["nmcli", "con", "modify", ssid, "802-11-wireless.mode", "ap",
                 "802-11-wireless.band", "bg", "ipv4.method", "shared"],
                ["nmcli", "con", "modify", ssid, "wifi-sec.key-mgmt", "wpa-psk",
                 "ipv4.addresses", "192.168.10.1/24"],
                ["nmcli", "con", "modify", ssid, "wifi-sec.psk", password],
            ]
            for cmd in cmds:
                subprocess.run(cmd, check=True)
        # Bring up the hotspot
        subprocess.run(["nmcli", "con", "up", ssid], check=True)
        print("Hotspot active.")
    except subprocess.CalledProcessError as e:
        print(f"Error activating hotspot: {e}")


def main() -> None:
    parser = argparse.ArgumentParser(description="K902 differential GNSS service for Orange Pi")
    parser.add_argument("--serial", default="/dev/ttyS3",
                        help="Serial device connected to K902 (default: /dev/ttyS3)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate for serial connection (default: 115200)")
    parser.add_argument("--nmea-port", type=int, default=5001,
                        help="TCP port to serve NMEA sentences (default: 5001)")
    # NTRIP parameters
    parser.add_argument("--ntrip-host", help="Hostname of NTRIP caster")
    parser.add_argument("--ntrip-port", type=int, help="Port of NTRIP caster (default: 2101)", default=2101)
    parser.add_argument("--ntrip-mountpoint", help="Mountpoint on NTRIP caster")
    parser.add_argument("--ntrip-user", help="Username for NTRIP caster (optional)")
    parser.add_argument("--ntrip-pass", help="Password for NTRIP caster (optional)")
    # Hotspot parameters
    parser.add_argument("--hotspot-ssid", help="SSID for Wi‑Fi hotspot (optional)")
    parser.add_argument("--hotspot-pass", default="12345678",
                        help="Password for Wi‑Fi hotspot (default: 12345678)")
    args = parser.parse_args()

    # Optionally activate Wi‑Fi hotspot
    if args.hotspot_ssid:
        activate_hotspot(args.hotspot_ssid, args.hotspot_pass)

    # Open serial port
    try:
        ser = serial.Serial(args.serial, args.baud, timeout=1)
    except Exception as e:
        print(f"Error opening serial port {args.serial}: {e}")
        return

    # Send initialisation commands for NMEA output
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
    print("Sending initialisation commands to K902…")
    send_initialization_commands(ser, init_cmds)
    print("Initialisation complete.")

    # Start NMEA TCP server
    server = NMEATcpServer(ser, port=args.nmea_port)
    server.start()

    # Start NTRIP client if parameters provided
    ntrip_thread = None
    if args.ntrip_host and args.ntrip_mountpoint:
        ntrip_thread = NTRIPClient(
            host=args.ntrip_host,
            port=args.ntrip_port,
            mountpoint=args.ntrip_mountpoint,
            username=args.ntrip_user,
            password=args.ntrip_pass,
            ser=ser
        )
        ntrip_thread.start()

    try:
        # Keep the main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping services…")
    finally:
        if ntrip_thread:
            ntrip_thread.stop()
        server.stop()
        ser.close()


if __name__ == "__main__":
    main()