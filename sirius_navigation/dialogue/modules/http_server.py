# -*- coding: utf-8 -*-
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

class ReusableHTTPServer(HTTPServer):
    allow_reuse_address = True

class HttpInstructionServer:
    def __init__(self, node):
        self.node = node

    def start(self):
        node_ref = self.node
        class InstructionHTTPHandler(BaseHTTPRequestHandler):
            node = node_ref
            
            def log_message(self, format, *args):
                pass # Suppress logs to stdout

            def do_POST(self):
                if self.path == '/instruction':
                    try:
                        content_length = int(self.headers['Content-Length'])
                        post_data = self.rfile.read(content_length)
                        data = json.loads(post_data.decode('utf-8'))
                        instruction = data.get('instruction', '')
                        if instruction:
                            self.node.get_logger().info(f'Received HTTP instruction: "{instruction}"')
                            threading.Thread(
                                target=self.node.process_instruction,
                                args=(instruction,),
                                daemon=True
                            ).start()
                            self.send_response(200)
                            self.send_header('Content-Type', 'application/json')
                            self.end_headers()
                            self.wfile.write(json.dumps({"success": True}).encode('utf-8'))
                            return
                    except Exception as e:
                        self.node.get_logger().error(f"Error handling HTTP instruction: {e}")
                
                self.send_response(400)
                self.end_headers()

        def run_server():
            server_address = ('', 50060)
            try:
                httpd = ReusableHTTPServer(server_address, InstructionHTTPHandler)
                self.node.get_logger().info("=== Instruction HTTP Server running on port 50060 ===")
                httpd.serve_forever()
            except Exception as e:
                self.node.get_logger().error(f"Failed to start Instruction HTTP Server: {e}")

        threading.Thread(target=run_server, daemon=True).start()
