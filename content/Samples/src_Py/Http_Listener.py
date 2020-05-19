import http.server
import socketserver
import random
import json
    
class I_Comm_handler():
    def handle_comm(self, request_body):
        return "null"

class Request_Handler(http.server.BaseHTTPRequestHandler):
    def do_POST(self):
        L = int(self.headers.get('content-length', 0))
        req = json.loads(self.rfile.read(L).decode("utf-8"))
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        k = req["KEY"] 
        if(k == 0):
            if(self.server.Bind_key == None):
                self.server.Bind_key = random.randint(1, 2000)
                self.__write_word(str(self.server.Bind_key))
                return
        elif(k == self.server.Bind_key):
                self.__write_word(self.server.Comm_resolver.handle_comm(req["BODY"]))
                return
        self.__write_word("null")

    def __write_word(self, str_word):
        to_send = bytearray(str_word, 'utf-8')
        self.wfile.write(to_send)

class HttpServer(socketserver.TCPServer):
    def __init__(self, PORT, comm_handler):
        super().__init__(("", PORT), Request_Handler)
        self.Bind_key = None
        self.Comm_resolver = comm_handler


def run_server(port, comm_hndlr:I_Comm_handler):
    httpd = HttpServer(port, comm_hndlr)
    httpd.serve_forever()
