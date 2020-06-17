import socket

class I_Stream_Socket:
    def __init__(self, address, port):
        self._address = address
        self._port = port
        self._socket = None
    
    def get_address(self):
        return self._address
        
    def get_port(self):
        return self._port

    def Recv_val(self):
        raw = self._socket.recv(4)
        return int.from_bytes(raw, 'little')

    def Send_val(self, val):
        #self._socket.sendall(val.to_bytes(4, 'little'))
        Bytes = bytearray(4)
        i = 0
        while(i < 4):
            Bytes[i] = (val >> 8 * i) & 0xFF
            i = i +1
        self._socket.sendall(Bytes)


    def Recv_word(self):
        S = self.Recv_val()
        raw = self._socket.recv(S)
        return raw.decode("utf-8") 

    def Send_word(self, Word):
        self.Send_val(Word.__len__())
        self._socket.sendall(bytearray(Word, 'utf-8'))



class Stream_to_Server(I_Stream_Socket):
    def __init__(self, address, port):
        super().__init__(address, port)

    def InitConnection(self):
        keep_try = True
        while(keep_try):
            keep_try = False
            try:
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._socket.connect((self._address, self._port))
            except:
                keep_try = True



class Stream_to_Client(I_Stream_Socket):
    def __init__(self, port):
        super().__init__('', port)

    def InitConnection(self):
        self.__socket_init =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__socket_init.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__socket_init.bind((self._address, self._port))
        self.__socket_init.listen(0)
        self._socket = self.__socket_init.accept()  

