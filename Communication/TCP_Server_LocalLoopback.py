"""
TCP Server implementation for receiving data from TCP clients.
Configured for local loopback communication.

@author: ioannisgeorgilas
"""
# First we import our libraries
import socket   # This library will allow you to communicate over the network
import logging  # This library will offer us a different method to print information on the terminal (better for debugging purposes)

# setting the logging level to INFO
logging.basicConfig(level=logging.INFO)

# This is the IP address that the server will listen on (localhost for loopback)
TCP_IP = "127.0.0.1"

# This is the port that the server will listen on
TCP_PORT = 25000

# Buffer size for receiving data
BUFFER_SIZE = 1024

# Create the socket for the TCP communication
s = socket.socket(socket.AF_INET,        # Family of addresses, in this case IP type 
                  socket.SOCK_STREAM)    # What protocol to use, in this case TCP (streamed communications)
logging.info('Socket successfully created')

# Bind the socket to the IP address and port
s.bind((TCP_IP, TCP_PORT))
logging.info(f'Socket bound to {TCP_IP}:{TCP_PORT}')

# Start listening for incoming connections (max 1 queued connection)
s.listen(1)
logging.info('Server is listening for connections...')

# Accept incoming connection
conn, addr = s.accept()
logging.info(f'Connection established with {addr}')

# Receive data from the client
while True:
    data = conn.recv(BUFFER_SIZE)
    if not data:
        break
    # Convert received bytes to list and display
    byte_values = list(data)
    logging.info(f'Received: {byte_values}')

# Close the connection and socket
conn.close()
s.close()
logging.info('Server closed')