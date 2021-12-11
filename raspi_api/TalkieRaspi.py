import socket

class TalkieRaspi():
    def __init__(self, server_address, port):
        self.server = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
                )
        try:
            print(F"Starting server at {server_address} in {port}")
            self.server.bind(
                    (server_address, port)
                    )
        except Exception as err:
            print(F"Error found:\n{err}")
            self.server.close()

    def start(self):
        self.server.listen(1)
        print(F"Wait for connection")
        data = connection.recv(5)
        if data == b"bye!!":
            print("good bye")
            connection.send(b'bye!!')
            connection.close()
            exit()
        print(F"received {data}")
        connection.send(b'hello')

    def close(self):
        self.server.close()

