#!/usr/bin/env python3
"""
k902_bt_web_server.py
---------------------

Esta utilidad combina varias funciones para convertir una Orange Pi
conectada a un módulo SinoGNSS K902 en una estación GNSS versátil que
puede ser configurada a través de un navegador web y transmitir datos
tanto por Wi‑Fi como por Bluetooth.  Incorpora:

* **Salida NMEA por TCP**: igual que en ``k902_dgps_server.py``, reenvía
  las frases NMEA leídas del K902 a todos los clientes conectados a un
  puerto TCP configurado.
* **Salida NMEA por Bluetooth**: abre un socket RFCOMM (Serial Port
  Profile) y, tras aceptar un dispositivo emparejado, envía las frases
  NMEA a ese cliente.  Esto permite conectar aplicaciones en el móvil
  mediante Bluetooth en lugar de TCP.  Es necesario emparejar el móvil
  con la Orange Pi y, posiblemente, registrar un servicio SPP con BlueZ.
* **Interfaz de configuración vía web**: un servidor HTTP muy simple
  (puerto 8080) expone una página donde se puede elegir si el K902
  trabaja en modo «rover» (NMEA) o «base» (RTCM).  En modo base se
  solicita la latitud, longitud y altura geodésica de la estación.
  Al enviar el formulario, el servidor envía los comandos apropiados
  al módulo K902 a través del puerto serie.

Este script no pretende reemplazar al software completo de ComNav, sino
proporcionar una solución ligera.  Requiere Python 3, pyserial y acceso
a las bibliotecas de Bluetooth del sistema (BlueZ) para poder crear el
socket RFCOMM.

Uso básico:

```
sudo python3 k902_bt_web_server.py --serial /dev/ttyS3 --baud 115200 \
    --nmea-port 5001 --bt-port 3
```

Luego accede a `http://<ip_de_la_orange>:8080/` desde el navegador para
configurar modo base o rover.  Empareja tu teléfono mediante
Bluetooth y conéctate al canal RFCOMM indicado para recibir NMEA.

Autor: ChatGPT
Licencia: MIT
"""

import argparse
import base64
import os
import socket
import subprocess
import threading
import time
import textwrap
from http import server
from urllib.parse import parse_qs
from typing import List, Optional

import serial


def send_initialization_commands(ser: serial.Serial, commands: List[bytes], delay: float = 0.2) -> None:
    for cmd in commands:
        ser.write(cmd)
        ser.flush()
        time.sleep(delay)


class NMEATcpServer:
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
        print(f"Servidor NMEA (TCP) escuchando en {self.host}:{self.port}")
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
                print(f"Cliente TCP conectado desde {addr}")
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


class BluetoothNMEAService(threading.Thread):
    """Servicio RFCOMM sencillo que envía NMEA al primer cliente emparejado.

    Para que funcione:
    - La Orange Pi debe disponer de Bluetooth y el kernel debe tener
      soporte RFCOMM.
    - El canal RFCOMM (``channel``) debe estar libre y, en algunos
      dispositivos, el servicio SPP debe estar registrado en el daemon
      Bluetooth (puedes hacerlo con `sdptool add SP`).
    - El cliente (teléfono) debe emparejarse con la Orange Pi antes de
      conectar al canal RFCOMM.
    """
    def __init__(self, ser: serial.Serial, channel: int = 3) -> None:
        super().__init__(daemon=True)
        self.ser = ser
        self.channel = channel
        self._stop_event = threading.Event()
        self.server_sock: Optional[socket.socket] = None
        self.client_sock: Optional[socket.socket] = None

    def stop(self) -> None:
        self._stop_event.set()
        try:
            if self.client_sock:
                self.client_sock.close()
        except Exception:
            pass
        try:
            if self.server_sock:
                self.server_sock.close()
        except Exception:
            pass

    def run(self) -> None:
        # Crear socket RFCOMM
        try:
            self.server_sock = socket.socket(socket.AF_BLUETOOTH,
                                            socket.SOCK_STREAM,
                                            socket.BTPROTO_RFCOMM)
            # Bind a todos los adaptadores locales, canal especificado
            self.server_sock.bind(("", self.channel))
            self.server_sock.listen(1)
            print(f"Esperando conexión Bluetooth en canal RFCOMM {self.channel}…")
            self.server_sock.settimeout(1.0)
            while not self._stop_event.is_set():
                try:
                    client, addr = self.server_sock.accept()
                    print(f"Cliente Bluetooth conectado desde {addr}")
                    self.client_sock = client
                    client.setblocking(False)
                    # Leer del puerto serie y enviar al cliente
                    while not self._stop_event.is_set():
                        line = self.ser.readline()
                        if not line:
                            continue
                        try:
                            client.sendall(line)
                        except Exception:
                            print("Cliente Bluetooth desconectado")
                            client.close()
                            self.client_sock = None
                            break
                except socket.timeout:
                    continue
        except Exception as e:
            print(f"Error en servicio Bluetooth: {e}")


class ConfigHTTPRequestHandler(server.BaseHTTPRequestHandler):
    """Manejador HTTP para configurar el modo del K902.

    Al procesar una solicitud GET muestra un formulario HTML que permite
    escoger entre modo rover (NMEA) y modo base (RTCM).  En modo base
    solicita la latitud, longitud y altura geodésicas.  Cuando se envía
    el formulario (POST), los parámetros se parsean y se generan los
    comandos correspondientes que se envían al puerto serie asociado al
    manejador.
    """
    ser: serial.Serial = None  # Será establecido desde fuera

    def _html(self, body: str) -> bytes:
        return f"""<!DOCTYPE html>
<html lang="en"><head>
    <meta charset="utf-8">
    <title>Configuración K902</title>
    <style>
      body {{ font-family: Arial, sans-serif; margin: 2em; }}
      label {{ display: block; margin-top: 1em; }}
      input[type=text] {{ width: 10em; }}
      .btn {{ margin-top: 1em; padding: 0.5em 1em; }}
    </style>
</head><body>
{body}
</body></html>""".encode()

    def do_GET(self) -> None:
        body = """
<h1>Configuración del módulo K902</h1>
<form method="POST">
  <label><input type="radio" name="mode" value="rover" checked> Modo Rover (NMEA)</label>
  <label><input type="radio" name="mode" value="base"> Modo Base (RTCM)</label>
  <div id="basefields" style="margin-left:1em;">
    <label>Latitud (grados decimales): <input type="text" name="lat"></label>
    <label>Longitud (grados decimales): <input type="text" name="lon"></label>
    <label>Altura ellipsoidal (m): <input type="text" name="h"></label>
  </div>
  <input class="btn" type="submit" value="Aplicar">
</form>
<script>
  // Mostrar/ocultar campos base
  function updateFields() {
    var mode = document.querySelector('input[name="mode"]:checked').value;
    document.getElementById('basefields').style.display = (mode === 'base') ? 'block' : 'none';
  }
  var radios = document.querySelectorAll('input[name="mode"]');
  radios.forEach(function(r) { r.addEventListener('change', updateFields); });
  updateFields();
</script>
"""
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(self._html(body))

    def do_POST(self) -> None:
        # Recoger datos del formulario
        length = int(self.headers.get('Content-Length', 0))
        data = self.rfile.read(length).decode()
        params = parse_qs(data)
        mode = params.get('mode', [''])[0]
        if mode == 'rover':
            # Comandos NMEA estándar para rover
            cmds = [
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
            send_initialization_commands(self.ser, cmds)
            message = "Modo rover activado. Se están emitiendo frases NMEA."
        elif mode == 'base':
            lat = params.get('lat', [''])[0]
            lon = params.get('lon', [''])[0]
            h = params.get('h', [''])[0]
            # Validar y construir comando fix position.  Si no se
            # proporcionan coordenadas, se usa fix auto.
            cmds: List[bytes] = []
            try:
                if lat and lon and h:
                    lat_f = float(lat)
                    lon_f = float(lon)
                    h_f = float(h)
                    cmds.append(f"fix position {lat_f} {lon_f} {h_f}\r\n".encode())
                else:
                    cmds.append(b"fix auto\r\n")
            except ValueError:
                cmds.append(b"fix auto\r\n")
            # Activar salida RTCM típica (1006, 1074, 1084, 1094, 1124)
            cmds += [
                b"unlogall com1\r\n",
                b"log com1 rtcm1006b ontime 10\r\n",
                b"log com1 rtcm1074b ontime 1\r\n",
                b"log com1 rtcm1084b ontime 1\r\n",
                b"log com1 rtcm1094b ontime 1\r\n",
                b"log com1 rtcm1124b ontime 1\r\n",
                b"saveconfig\r\n",
            ]
            send_initialization_commands(self.ser, cmds)
            message = "Modo base activado. Se están emitiendo mensajes RTCM."
        else:
            message = "Modo desconocido."
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(self._html(f"<p>{message}</p><p><a href='/'>Volver</a></p>"))


class ConfigHTTPServer:
    def __init__(self, ser: serial.Serial, host: str = "0.0.0.0", port: int = 8080) -> None:
        self.ser = ser
        self.host = host
        self.port = port
        self.httpd: Optional[server.HTTPServer] = None
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        handler_class = ConfigHTTPRequestHandler
        handler_class.ser = self.ser
        self.httpd = server.HTTPServer((self.host, self.port), handler_class)
        print(f"Servidor web de configuración escuchando en http://{self.host}:{self.port}/")
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        if self.httpd:
            self.httpd.shutdown()
        if self.thread:
            self.thread.join()


def setup_access_point(interface: str = "wlan0",
                       ap_ip: str = "192.168.50.1/24",
                       dhcp_range: str = "192.168.50.10,192.168.50.50,12h",
                       ssid: str = "GNSS-K902",
                       password: str = "12345678",
                       start_script: str = "~/start_ap.sh",
                       service_name: str = "ap.service") -> None:
    """Configure and enable a Wi‑Fi access point using hostapd and dnsmasq.

    This helper creates the necessary configuration files and systemd
    service to turn the specified wireless interface into a static
    access point.  It follows the steps described in the setup
    document you provided: assign a static IP, configure dnsmasq for
    DHCP, configure hostapd for WPA2, create a startup script and a
    corresponding systemd unit.  Root privileges are required to
    modify files in /etc and reload systemd.

    :param interface: Name of the Wi‑Fi interface (default: wlan0)
    :param ap_ip: Static IP address/mask for the AP (default: 192.168.50.1/24)
    :param dhcp_range: DHCP range for dnsmasq (default: 192.168.50.10,192.168.50.50,12h)
    :param ssid: SSID of the hotspot
    :param password: WPA2 passphrase
    :param start_script: Path to the shell script that will start hostapd and dnsmasq
    :param service_name: Name of the systemd service to create
    """
    # Paths
    dnsmasq_conf = "/etc/dnsmasq.conf"
    hostapd_conf = "/etc/hostapd/hostapd.conf"
    start_script_path = os.path.expanduser(start_script)
    service_path = f"/etc/systemd/system/{service_name}"

    print("Configurando punto de acceso Wi‑Fi…")
    # Stop conflicting services
    subprocess.run(["systemctl", "stop", "NetworkManager"], check=False)
    subprocess.run(["systemctl", "stop", "wpa_supplicant"], check=False)
    subprocess.run(["systemctl", "stop", "systemd-resolved"], check=False)

    # Bring interface down/up and assign IP
    subprocess.run(["ip", "link", "set", interface, "down"], check=False)
    subprocess.run(["ip", "addr", "flush", "dev", interface], check=False)
    subprocess.run(["ip", "link", "set", interface, "up"], check=False)
    subprocess.run(["iw", "dev", interface, "set", "power_save", "off"], check=False)
    subprocess.run(["ip", "addr", "add", ap_ip, "dev", interface], check=False)

    # Write dnsmasq.conf
    dnsmasq_content = f"interface={interface}\n" \
                      f"dhcp-range={dhcp_range}\n" \
                      f"dhcp-option=3,{ap_ip.split('/')[0]}\n" \
                      f"dhcp-option=6,{ap_ip.split('/')[0]}\n"
    with open(dnsmasq_conf, "w") as f:
        f.write(dnsmasq_content)
    # Write hostapd.conf
    hostapd_content = f"interface={interface}\n" \
                      f"driver=nl80211\n" \
                      f"ssid={ssid}\n" \
                      "hw_mode=g\n" \
                      "channel=6\n" \
                      "auth_algs=1\n" \
                      "wpa=2\n" \
                      f"wpa_passphrase={password}\n" \
                      "wpa_key_mgmt=WPA-PSK\n" \
                      "rsn_pairwise=CCMP\n" \
                      "ignore_broadcast_ssid=0\n"
    os.makedirs(os.path.dirname(hostapd_conf), exist_ok=True)
    with open(hostapd_conf, "w") as f:
        f.write(hostapd_content)
    # Write start script
    script_content = textwrap.dedent(f"""
        #!/bin/bash
        # Configurar {interface} y arrancar dnsmasq/hostapd
        sudo ip link set {interface} down
        sudo ip addr flush dev {interface}
        sudo ip link set {interface} up
        sudo iw dev {interface} set power_save off
        sudo ip addr add {ap_ip} dev {interface}
        sudo dnsmasq -C {dnsmasq_conf}
        sudo hostapd {hostapd_conf}
    """)
    with open(start_script_path, "w") as f:
        f.write(script_content)
    os.chmod(start_script_path, 0o755)
    # Write systemd service
    service_content = textwrap.dedent(f"""
        [Unit]
        Description=Access Point Service
        After=network.target

        [Service]
        ExecStart={start_script_path}
        Restart=always
        User=root

        [Install]
        WantedBy=multi-user.target
    """)
    with open(service_path, "w") as f:
        f.write(service_content)
    # Reload and enable service
    subprocess.run(["systemctl", "daemon-reload"], check=False)
    subprocess.run(["systemctl", "enable", service_name], check=False)
    subprocess.run(["systemctl", "start", service_name], check=False)
    print("Punto de acceso configurado y servicio iniciado.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Servidor NMEA por TCP/Bluetooth y configuración web para K902")
    parser.add_argument("--serial", default="/dev/ttyS3", help="Dispositivo serie conectado al K902 (por defecto /dev/ttyS3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate para la conexión serie (por defecto 115200)")
    parser.add_argument("--nmea-port", type=int, default=5001, help="Puerto TCP para la salida NMEA (por defecto 5001)")
    parser.add_argument("--bt-port", type=int, default=3, help="Canal RFCOMM para la salida Bluetooth (por defecto 3)")
    # AP setup options
    parser.add_argument("--setup-ap", action="store_true", help="Configurar punto de acceso Wi‑Fi antes de iniciar el servidor")
    parser.add_argument("--ap-ssid", default="GNSS-K902", help="SSID del punto de acceso Wi‑Fi")
    parser.add_argument("--ap-pass", default="12345678", help="Contraseña del punto de acceso Wi‑Fi")
    parser.add_argument("--ap-ip", default="192.168.50.1/24", help="IP estática/máscara del punto de acceso (por defecto 192.168.50.1/24)")
    parser.add_argument("--ap-dhcp-range", default="192.168.50.10,192.168.50.50,12h", help="Rango DHCP para dnsmasq")
    parser.add_argument("--ap-interface", default="wlan0", help="Interfaz Wi‑Fi a usar para el AP (por defecto wlan0)")
    args = parser.parse_args()

    # If requested, set up the access point
    if args.setup_ap:
        setup_access_point(interface=args.ap_interface,
                           ap_ip=args.ap_ip,
                           dhcp_range=args.ap_dhcp_range,
                           ssid=args.ap_ssid,
                           password=args.ap_pass)

    try:
        ser = serial.Serial(args.serial, args.baud, timeout=1)
    except Exception as e:
        print(f"No se pudo abrir el puerto {args.serial}: {e}")
        return

    # Enviar comandos por defecto para activar NMEA inicialmente
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
    print("Inicializando K902 en modo rover por defecto…")
    send_initialization_commands(ser, init_cmds)

    # Servidor NMEA por TCP
    tcp_server = NMEATcpServer(ser, port=args.nmea_port)
    tcp_server.start()

    # Servicio NMEA por Bluetooth
    bt_service = BluetoothNMEAService(ser, channel=args.bt_port)
    bt_service.start()

    # Servidor HTTP para configuración
    http_server = ConfigHTTPServer(ser)
    http_server.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Deteniendo servicios…")
    finally:
        bt_service.stop()
        tcp_server.stop()
        http_server.stop()
        ser.close()


if __name__ == "__main__":
    main()