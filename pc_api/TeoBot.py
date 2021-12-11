import socket

class TeoBot():
    def __init__(self, server_address, port):
        robot = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
                )
        self.robot.settimeout(5)
        self.address = server_address
        self.port = port
        try:
            print(F"Connecting to {server_address} in {port}")
            self.robot.connect(
                    (server_address, port)
                    )
            print(F"Connection OK!")
        except Exception as err:
            print(F"Connection ERROR:\n{err}")

    def close(self):
        """ method used to close the connection with the client
        """
        print("Closing connection with robot")
        self.robot.close()

    def button(self, command):
        """ method that sends a commands and dont wait for an response
        """
        print("Sending button command")
        command = F"{command}!"
        command_b = str.encode(command)
        self.robot.send(command_b)

    def request(self, command):
        command = F"{command}!"
        command_b = str.encode(command)
        self.robot.send(command_b)
        return self.robot.recv(5)

    def stream(self):
        #this method receive a stream of all data and sends to ros
        command = b'give_it_to_me!'
        self.robot.send(command)
        return self.robot.recv(5)



