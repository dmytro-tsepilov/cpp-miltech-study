#!/usr/bin/env python3
"""Simple HTTP server to receive and log reports from perimeter miner system."""
from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import time
import sys

class ReportHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length)
        
        print(f"\n{'='*60}")
        print(f"[{time.strftime('%H:%M:%S')}] POST {self.path}")
        
        try:
            data = json.loads(body.decode())
            print(f"Body (formatted):")
            print(json.dumps(data, indent=2))
        except json.JSONDecodeError:
            print(f"Body (raw): {body.decode()}")
        
        print(f"{'='*60}")
        sys.stdout.flush()
        
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(b'{"status": "ok"}')
    
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(b'{"server": "perimeter-miner-http-reporter", "status": "running"}')
    
    def log_message(self, format, *args):
        pass  # Suppress default logging

if __name__ == '__main__':
    port = 8080
    server = HTTPServer(('0.0.0.0', port), ReportHandler)
    print(f"HTTP Reporter Server running on port {port}...")
    print("Waiting for reports from perimeter_miner system...")
    sys.stdout.flush()
    server.serve_forever()
