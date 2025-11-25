#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

This is an example to setup a UDP Listener.

@author: ioannisgeorgilas
"""

# First we import our libraries
import socket   # This library will allow you to communicate over the network
import sys      # This library will give us some information about your system

# First we need to set the IP and PORT we are going to listen to
# This is the localhost IP address (this machine)
UDP_IP = "127.0.0.1"
 
# This is the LOCAL port I am expecting data (on the sending machine this is the REMOTE port)
UDP_PORT = 50001

# Create the socket for the UDP communication
sock = socket.socket(socket.AF_INET,    # Family of addresses, in this case IP (Internet Protocol) family 
                     socket.SOCK_DGRAM) # What protocol to use, in this case UDP (datagram)

# Bind to the socket and wait for data on this port
sock.bind((UDP_IP, UDP_PORT))
print("Listening on IP:", UDP_IP, "Port:", UDP_PORT)

# Wait indifenetly (you will need to use Ctrl+C to stop the program)
while True:
    #Read data
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    # Print data
    print ("received message:", data.decode('utf-8')) # As a string (check the ASCII table)
    print ("received bytes:", list(data)) # As byte values
    print ("from address:", addr) # Print the address of the sender