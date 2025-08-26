#!/usr/bin/env python3
import argparse, socket, threading, time, serial, http.server, socketserver, subprocess, sys

def ensure_hotspot(ssid="GNSS-K902", password="12345678", ifname="wlan0", wifi_backend="nmcli"):
    if wifi_backend == "none":
        print("Hotspot deshabilitado (--wifi none)")
        return
    try:
        if wifi_backend == "nmcli":
            subprocess.run(["nmcli", "dev", "wifi", "hotspot", "ifname", ifname, "ssid", ssid, "password", password], check=True)
        else:
            print(f"Backend WiFi {wifi_backend} no soportado.")
    except Exception as e:
        print("Error al configurar el hotspot:", e)

def nmea_reader(port, baud, listeners):
    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            while True:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    for conn in listeners.copy():
                        try:
                            conn.sendall((line + "\n").encode())
                        except:
                            listeners.remove(conn)
    except Exception as e:
        print("Error leyendo NMEA:", e)

def tcp_server(host, port, listeners):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(5)
    print(f"Servidor NMEA TCP en {host}:{port}")
    while True:
        conn, _ = srv.accept()
        listeners.add(conn)

def http_server(port):
    class Handler(http.server.SimpleHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(b"<html><head><title>K902 GNSS</title></head><body>")
            self.wfile.write(b"<h1>Servidor GNSS K902</h1>")
            self.wfile.write(b"<p>Puerto NMEA TCP activo</p>")
            self.wfile.write(b"</body></html>")

    with socketserver.TCPServer(("", port), Handler) as httpd:
        print(f"Servidor web en http://0.0.0.0:{port}/")
        httpd.serve_forever()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", required=True, help="Puerto serial (ej. /dev/ttyS1)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--nmea-port", type=int, default=5001)
    parser.add_argument("--web-port", type=int, default=8080)
    parser.add_argument("--hotspot-password", default="12345678")
    parser.add_argument("--wifi", default="nmcli", help="Backend WiFi (nmcli|none)")
    args = parser.parse_args()

    ensure_hotspot("GNSS-K902", args.hotspot_password, "wlan0", args.wifi)

    listeners = set()
    threading.Thread(target=nmea_reader, args=(args.serial, args.baud, listeners), daemon=True).start()
    threading.Thread(target=tcp_server, args=("0.0.0.0", args.nmea_port, listeners), daemon=True).start()
    http_server(args.web_port)
