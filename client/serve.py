#!/usr/bin/env python3
"""Simple HTTP server to serve the WebRTC test client."""

import http.server
import socketserver
import os
import argparse

def main():
    parser = argparse.ArgumentParser(description='Serve WebRTC test client')
    parser.add_argument('-p', '--port', type=int, default=8000,
                        help='Port to serve on (default: 8000)')
    args = parser.parse_args()

    # Change to the client directory
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    handler = http.server.SimpleHTTPRequestHandler

    with socketserver.TCPServer(("0.0.0.0", args.port), handler) as httpd:
        import socket
        hostname = socket.gethostname()
        try:
            local_ip = socket.gethostbyname(hostname)
        except:
            local_ip = "unknown"
        print(f"Serving client at:")
        print(f"  Local:   http://localhost:{args.port}")
        print(f"  Network: http://{local_ip}:{args.port}")
        print("Press Ctrl+C to stop")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nStopping server...")

if __name__ == '__main__':
    main()
