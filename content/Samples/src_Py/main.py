from Http_Listener import I_Comm_handler
from Http_Listener import run_server
from Stream_Socket import Stream_to_Server
import json
import math
import tkinter as tk
from tkinter import filedialog


Cpp_Conn =  Stream_to_Server('127.0.0.1' , 2001)
Cpp_Conn.InitConnection()
print("socket connection done")

def Ask_file():
    root = tk.Tk()
    root.withdraw()
    return filedialog.askopenfilename()
def Ask_folder():
    root = tk.Tk()
    root.withdraw()
    return filedialog.asksaveasfile()

class Read_handler(I_Comm_handler):
    def handle_comm(self, request):
        request = str(request)
        request = request.replace("'", '"')
        request = request.replace("None", 'null')
        J_req = json.loads(request)
        nam = J_req['N']

        if(nam == "kill"):
            exit()
        elif(nam == "exp"):
            location = Ask_folder()
            f = open(location.name, "w")
            f.write(json.dumps(J_req['B']))
            f.close()
            return ""
        elif(nam == "imp"):
            location = Ask_file()
            f = open(location, "r")
            return f.read()
        else:
            Cpp_Conn.Send_word(nam)
            body = json.dumps(J_req['B'])
            Cpp_Conn.Send_word(body)
            to_send = Cpp_Conn.Recv_word()
            return to_send


comm = Read_handler()
run_server(500 , comm)

