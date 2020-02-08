import socket


class TCPSocketServer:
    def __init__(self, address='', port=4061, max_connections=5):
        # next create a socket object 
        self.s = socket.socket()	
        self.port = port

        # Bind address to port.
        self.s.bind((address, port))
        
        # Wair or incoming connections
        self.s.listen(max_connections)	 

    def write(self):
        pass
    
    def read(self):
        pass

    def start(self):
        while True: 
            c, addr = self.s.accept()	 

			
	

