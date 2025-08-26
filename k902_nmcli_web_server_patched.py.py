#!/usr/bin/env python3
"""
k902_nmcli_web_server.py
------------------------

Este script combina la funcionalidad de servidor GNSS con la gestión de la
Wi‑Fi mediante `nmcli`. Está pensado para un escenario en el que el
usuario no sabe de antemano a qué red se conectará la Orange Pi cuando
funcione como base, pero quiere poder seleccionar esa red desde un
navegador a través del hotspot inicial. La secuencia de uso es la
siguiente:

1. Al arrancar, la Orange Pi crea un hotspot llamado "Hotspot" en la
   interfaz `wlan0` usando NetworkManager. El móvil se conecta a ese
   SSID y accede a `http://10.42.0.1:8080/` (o la IP que asigne
   NetworkManager) para ver la interfaz de configuración.
2. En la página web se ofrecen dos opciones: «Rover» y «Base». En modo
   rover el K902 se configura para emitir NMEA y el hotspot permanece
   activo. En modo base el usuario selecciona una red Wi‑Fi disponible y
   aporta la contraseña; el script desactiva el hotspot y se conecta a
   esa red para tener salida a Internet antes de configurar el K902 como
   base (RTCM).

Esta implementación utiliza los mismos comandos GNSS que en los scripts
anteriores y no asigna una IP estática a la interfaz Wi‑Fi; toda la
gestión de IP, DHCP y NAT la realiza NetworkManager con el método
`shared`. Se requiere tener instalado `network-manager` y `nmcli` en
la Orange Pi.

Autor: ChatGPT
Licencia: MIT
"""

import argparse
import socket
import subprocess
import threading
import time
from http import server
from urllib.parse import parse_qs
from typing import List, Optional

import serial


def ensure_hotspot(ssid: str = "Hotspot", password: str = "12345678", ifname: str = "wlan0", wifi_backend=args.wifi) -> None:
    """Create or activate a Wi‑Fi hotspot using NetworkManager.

    If a connection with the given SSID already exists, it will be
    brought up. Otherwise it will be created with NAT/sharing enabled.
    """
    # Check if the connection exists
    try:
        result = subprocess.run([
            "nmcli", "-t", "-f", "NAME", "connection", "show"
        ], capture_output=True, text=True, check=True)
        names = result.stdout.strip().split("\n")
        if ssid not in names:
            # Create connection
            subprocess.run([
                "nmcli", "connection", "add", "type", "wifi",
                "ifname", ifname,
                "con-name", ssid,
                "autoconnect", "no",
                "ssid", ssid
            ], check=True)
            subprocess.run([
                "nmcli", "connection", "modify", ssid,
                "802-11-wireless.mode", "ap",
                "802-11-wireless.band", "bg",
                "ipv4.method", "shared"
            ], check=True)
            subprocess.run([
                "nmcli", "connection", "modify", ssid,
                "wifi-sec.key-mgmt", "wpa-psk",
                "wifi-sec.psk", password
            ], check=True)
        # Bring up the hotspot
        subprocess.run(["nmcli", "connection", "up", ssid], check=True)
        print(f"Hotspot '{ssid}' activo.")
    except Exception as e:
        print(f"Error al configurar el hotspot: {e}")


def connect_to_wifi(ssid: str, password: str, ifname: str = "wlan0") -> bool:
    """Connect to a Wi‑Fi network using nmcli. Returns True on success."""
    try:
        # Bring down hotspot if active
        subprocess.run(["nmcli", "connection", "down", "Hotspot"], check=False)
        # Connect to target network
        result = subprocess.run([
            "nmcli", "device", "wifi", "connect", ssid, "password", password, "ifname", ifname
        ], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"Conectado a la red {ssid}")
            return True
        else:
            print(f"No se pudo conectar a {ssid}: {result.stderr}")
            return False
    except Exception as e:
        print(f"Error al conectar a Wi‑Fi: {e}")
        return False


def scan_wifi(ifname: str = "wlan0") -> List[str]:
    """Return a list of available SSIDs using nmcli."""
    try:
        result = subprocess.run([
            "nmcli", "-t", "-f", "SSID", "device", "wifi", "list", "ifname", ifname
        ], capture_output=True, text=True)
        networks = []
        for line in result.stdout.strip().split("\n"):
            if line:
                networks.append(line)
        # Remove duplicates
        uniq = []
        [uniq.append(x) for x in networks if x not in uniq]
        return uniq
    except Exception as e:
        print(f"Error al escanear redes Wi‑Fi: {e}")
        return []


class NMEATcpServer:
    """Broadcast NMEA lines over TCP to connected clients."""
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
        print(f"Servidor NMEA TCP en {self.host}:{self.port}")
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
                client.close()
            except Exception:
                pass
        self.clients.clear()

    def _accept_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                client, addr = self.server_sock.accept()
                print(f"Cliente conectado {addr}")
                client.setblocking(False)
                self.clients.append(client)
            except Exception:
                time.sleep(0.1)

    def _broadcast_loop(self) -> None:
        while not self._stop_event.is_set():
            line = self.ser.readline()
            if not line:
                continue
            disconnected = []
            for client in self.clients:
                try:
                    client.sendall(line)
                except Exception:
                    disconnected.append(client)
            for c in disconnected:
                try:
                    c.close()
                except Exception:
                    pass
                try:
                    self.clients.remove(c)
                except Exception:
                    pass


class NtripPushClient:
    """Cliente que envía correcciones RTCM a un caster NTRIP.

    Este hilo se conecta a un servidor NTRIP para publicar datos. Una vez
    establecida la conexión con la cabecera `SOURCE password mountpoint`,
    lee bytes del puerto serie y los reenvía al caster. Está pensado
    para usarse en modo base: el puerto serie debe estar configurado
    para emitir mensajes RTCM. Si la conexión se cae, se intenta
    reconectar cada pocos segundos.
    """

    def __init__(self, ser: serial.Serial, host: str, port: int, mountpoint: str, password: str, agent: str = "NTRIP Python Push/1.0") -> None:
        self.ser = ser
        self.host = host
        self.port = port
        self.mountpoint = mountpoint
        self.password = password
        self.agent = agent
        self._stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            sock: Optional[socket.socket] = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(10)
                sock.connect((self.host, self.port))
                # Enviar cabecera SOURCE
                header = f"SOURCE {self.password} {self.mountpoint}\r\nSource-Agent: {self.agent}\r\n\r\n"
                sock.sendall(header.encode())
                # No esperamos respuesta; algunos casters devuelven "ICY 200 OK"
                sock.settimeout(None)
                print(f"Conectado al caster NTRIP en {self.host}:{self.port} para {self.mountpoint}")
                # Bucle de envío: leer bytes del puerto serie y enviar
                self.ser.timeout = 0.2
                while not self._stop_event.is_set():
                    try:
                        data = self.ser.read(4096)
                    except Exception:
                        data = b""
                    if data:
                        try:
                            sock.sendall(data)
                        except Exception as e:
                            print(f"Error enviando a caster NTRIP: {e}")
                            break
                    else:
                        time.sleep(0.1)
                print("Terminando hilo NTRIP push…")
            except Exception as e:
                print(f"No se pudo conectar al caster NTRIP: {e}")
            finally:
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass
            # Esperar antes de reintentar si no se solicitó detener
            if not self._stop_event.is_set():
                time.sleep(5)


class ConfigHTTPRequestHandler(server.BaseHTTPRequestHandler):
    """HTTP handler que gestiona la configuración del módulo.

    Se utilizan atributos de clase para inyectar dependencias desde
    ConfigHTTPServer: ``ser`` (instancia de serial.Serial), ``nmea_server``
    (instancia de NMEATcpServer) y ``ntrip_push`` (instancia de
    NtripPushClient o None). Esto permite que las peticiones POST puedan
    arrancar o detener servicios según el modo seleccionado por el
    usuario.
    """

    ser: serial.Serial = None  # puerto serie del K902, inyectado
    nmea_server: Optional['NMEATcpServer'] = None  # servidor NMEA actual
    ntrip_push: Optional['NtripPushClient'] = None  # cliente de subida NTRIP
    nmea_port: int = 5001  # puerto TCP para NMEA (inyectado desde main)

    def _html(self, body: str) -> bytes:
        return ("""
<!DOCTYPE html>
<html lang="es">
<head><meta charset="utf-8"><title>Configuración GNSS</title>
<style>
body { font-family: sans-serif; margin: 1em; }
.networks { margin-top: 0.5em; }
.hidden { display: none; }
</style>
</head><body>
""" + body + "\n</body></html>").encode()

    def do_GET(self) -> None:
        """Genera la página HTML con la lista de redes Wi‑Fi y los campos
        necesarios para configurar el modo rover o base.

        La sección de configuración de la base incluye campos para
        coordenadas y parámetros NTRIP. Un pequeño script en el
        cliente muestra u oculta las opciones dependiendo del modo
        seleccionado.
        """
        nets = scan_wifi()
        # Construimos opciones para la lista de redes
        options = ''.join(f"<option value='{n}'>{n}</option>" for n in nets)
        # Escribimos el cuerpo del formulario. Evitamos f-strings
        # anidados para simplificar la sintaxis y la generación.
        body = (
            "<h1>Modo de funcionamiento</h1>"
            "<form method='POST'>"
            "  <label><input type='radio' name='mode' value='rover' checked> Rover (Hotspot activo)</label><br>"
            "  <label><input type='radio' name='mode' value='base'> Base (conectar a red Wi‑Fi)</label><br><br>"
            "  <div id='wifi_select' class='hidden'>"
            "    <label>SSID:&nbsp;<select name='ssid'>"
            "      <option value=''>--Seleccionar--</option>"
            + options +
            "    </select></label><br>"
            "    <label>Contraseña de Wi‑Fi:&nbsp;<input type='password' name='wifipass'></label><br><br>"
            "    <label>Latitud:&nbsp;<input type='text' name='lat' placeholder='ej. -6.1234'></label><br>"
            "    <label>Longitud:&nbsp;<input type='text' name='lon' placeholder='ej. -79.5678'></label><br>"
            "    <label>Altura (m):&nbsp;<input type='text' name='hgt' placeholder='ej. 10.0'></label><br><br>"
            "    <label>Host NTRIP:&nbsp;<input type='text' name='ntrip_host' placeholder='ej. caster.example.com'></label><br>"
            "    <label>Puerto NTRIP:&nbsp;<input type='text' name='ntrip_port' placeholder='2101'></label><br>"
            "    <label>Mountpoint:&nbsp;<input type='text' name='ntrip_mp' placeholder='MOUNT'></label><br>"
            "    <label>Contraseña NTRIP:&nbsp;<input type='text' name='ntrip_pass' placeholder='clave'></label><br>"
            "  </div>"
            "  <button type='submit'>Aplicar</button>"
            "</form>"
            "<script>\n"
            "function update() {\n"
            "  var mode = document.querySelector('input[name="mode"]:checked').value;\n"
            "  document.getElementById('wifi_select').style.display = (mode === 'base') ? 'block' : 'none';\n"
            "}\n"
            "var radios = document.querySelectorAll('input[name="mode"]');\n"
            "radios.forEach(function(r) { r.addEventListener('change', update); });\n"
            "update();\n"
            "</script>"
        )
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(self._html(body))

    def do_POST(self) -> None:
        """Procesa el formulario enviado por el usuario.

        Dependiendo del modo elegido (rover o base) se configura el
        módulo K902, se gestiona la interfaz de red (hotspot o cliente)
        y se arrancan o detienen los servicios de difusión NMEA y
        subida NTRIP.
        """
        length = int(self.headers.get('Content-Length', 0))
        data = self.rfile.read(length).decode()
        params = parse_qs(data)
        mode = params.get('mode', [''])[0]
        message = ""
        # Detener cliente NTRIP si estuviese corriendo
        if ConfigHTTPRequestHandler.ntrip_push is not None:
            ConfigHTTPRequestHandler.ntrip_push.stop()
            ConfigHTTPRequestHandler.ntrip_push = None
        if mode == 'rover':
            # Activar hotspot y desconectar de redes
            ensure_hotspot()
            # Asegurarse de que el servidor NMEA está activo
            if ConfigHTTPRequestHandler.nmea_server is None:
                # Crear y arrancar un nuevo servidor NMEA sobre el puerto
                ConfigHTTPRequestHandler.nmea_server = NMEATcpServer(
                    ConfigHTTPRequestHandler.ser, port=ConfigHTTPRequestHandler.nmea_port
                )
                ConfigHTTPRequestHandler.nmea_server.start()
            # Configurar K902 en modo rover (NMEA)
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
            for cmd in cmds:
                ConfigHTTPRequestHandler.ser.write(cmd)
                ConfigHTTPRequestHandler.ser.flush()
                time.sleep(0.2)
            message = "Módulo configurado en modo rover. Hotspot activo."
        elif mode == 'base':
            # Recuperar campos del formulario
            ssid = params.get('ssid', [''])[0]
            wifipass = params.get('wifipass', [''])[0]
            lat = params.get('lat', [''])[0]
            lon = params.get('lon', [''])[0]
            hgt = params.get('hgt', [''])[0]
            ntrip_host = params.get('ntrip_host', [''])[0]
            ntrip_port = params.get('ntrip_port', [''])[0]
            ntrip_mp = params.get('ntrip_mp', [''])[0]
            ntrip_pass = params.get('ntrip_pass', [''])[0]
            if not ssid:
                message = "Debes seleccionar una red Wi‑Fi para modo base."
            else:
                if connect_to_wifi(ssid, wifipass):
                    # Apagar el hotspot manualmente, en caso de que siga activo
                    subprocess.run(["nmcli", "connection", "down", "Hotspot"], check=False)
                    # Parar servidor NMEA para liberar el puerto serie en modo base
                    if ConfigHTTPRequestHandler.nmea_server is not None:
                        ConfigHTTPRequestHandler.nmea_server.stop()
                        ConfigHTTPRequestHandler.nmea_server = None
                    # Configurar K902 como base con o sin coordenadas
                    cmds = []
                    # Si el usuario proporcionó todas las coordenadas
                    if lat and lon and hgt:
                        try:
                            float(lat); float(lon); float(hgt)
                            cmds.append(f"fix position {lat} {lon} {hgt}\r\n".encode())
                        except ValueError:
                            # Si las coordenadas no son válidas, usar auto
                            cmds.append(b"fix auto\r\n")
                    else:
                        cmds.append(b"fix auto\r\n")
                    cmds += [
                        b"unlogall com1\r\n",
                        b"log com1 rtcm1006b ontime 10\r\n",
                        b"log com1 rtcm1074b ontime 1\r\n",
                        b"log com1 rtcm1084b ontime 1\r\n",
                        b"log com1 rtcm1094b ontime 1\r\n",
                        b"log com1 rtcm1124b ontime 1\r\n",
                        b"saveconfig\r\n",
                    ]
                    for cmd in cmds:
                        ConfigHTTPRequestHandler.ser.write(cmd)
                        ConfigHTTPRequestHandler.ser.flush()
                        time.sleep(0.2)
                    # Si hay parámetros NTRIP, arrancar subida
                    if ntrip_host and ntrip_port and ntrip_mp and ntrip_pass:
                        try:
                            ntrip_port_int = int(ntrip_port)
                        except ValueError:
                            ntrip_port_int = 2101
                        ConfigHTTPRequestHandler.ntrip_push = NtripPushClient(
                            ConfigHTTPRequestHandler.ser,
                            ntrip_host,
                            ntrip_port_int,
                            ntrip_mp,
                            ntrip_pass
                        )
                        ConfigHTTPRequestHandler.ntrip_push.start()
                        message = f"Módulo configurado en modo base, conectado a {ssid} y enviando RTCM a {ntrip_host}:{ntrip_port}/{ntrip_mp}."
                    else:
                        message = f"Módulo configurado en modo base y conectado a {ssid}. No se configuró caster NTRIP."
                else:
                    message = f"No se pudo conectar a la red {ssid}."
        else:
            message = "Modo desconocido."
        # Respuesta HTML
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(self._html(f"<p>{message}</p><p><a href='/'>&laquo; Volver</a></p>"))


class ConfigHTTPServer:
    def __init__(self, ser: serial.Serial, host: str = "0.0.0.0", port: int = 8080) -> None:
        self.ser = ser
        self.host = host
        self.port = port
        handler_class = ConfigHTTPRequestHandler
        # Inyectamos el puerto serie y ponemos referencias iniciales a
        # servidores para que puedan ser controlados desde la interfaz web.
        handler_class.ser = ser
        handler_class.nmea_server = None
        handler_class.ntrip_push = None
        self.httpd = server.HTTPServer((self.host, self.port), handler_class)
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    def start(self) -> None:
        print(f"Servidor web en http://{self.host}:{self.port}/")
        self.thread.start()

    def stop(self) -> None:
        self.httpd.shutdown()
        self.thread.join()


def main() -> None:
    parser = argparse.ArgumentParser(description="Servidor GNSS con hotspot y selección de Wi‑Fi")
    
parser.add_argument("--wifi", choices=["nmcli","hostapd","none"], default="nmcli", help="Backend Wi‑Fi: nmcli (NetworkManager), hostapd (manual), none")
parser.add_argument("--serial", default="/dev/ttyS3", help="Dispositivo serie del K902")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate del puerto serie")
    parser.add_argument("--nmea-port", type=int, default=5001, help="Puerto TCP para NMEA")
    parser.add_argument("--hotspot-password", default="12345678", help="Contraseña del hotspot inicial")
    args = parser.parse_args()

    # Abrir puerto serie
    try:
        ser = serial.Serial(args.serial, args.baud, timeout=1)
    except Exception as e:
        print(f"No se pudo abrir el puerto {args.serial}: {e}")
        return

    # Asegurar hotspot activo
    ensure_hotspot(password=args.hotspot_password)

    # Inicializar módulo como rover por defecto
    initial_cmds = [
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
    for cmd in initial_cmds:
        ser.write(cmd)
        ser.flush()
        time.sleep(0.2)

    # Servidor NMEA TCP
    nmea_server = NMEATcpServer(ser, port=args.nmea_port)
    nmea_server.start()

    # Servidor HTTP de configuración
    http_server = ConfigHTTPServer(ser)
    # Inyectar el servidor NMEA en el manejador para que pueda ser
    # gestionado desde la interfaz web
    ConfigHTTPRequestHandler.nmea_server = nmea_server
    ConfigHTTPRequestHandler.nmea_port = args.nmea_port
    http_server.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Deteniendo…")
    finally:
        http_server.stop()
        nmea_server.stop()
        ser.close()


if __name__ == "__main__":
    main()