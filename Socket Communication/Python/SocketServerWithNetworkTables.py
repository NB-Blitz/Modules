#This is only a starting point for socket communication.  Error handling needs to
#be performed, such as checking for error values returns from function calls and
#try/catch exception handling

#This socket handling should be threaded to allow for one thread to just handle vision
#processing while another thread handles socket communication

import socket
import select


#The NetworkTables allows for data to be placed on the SmartDashboard.  We can use this to
#help debug or show the status of the PI.  Don't use the network tables to return data for
#vision processing to be used for control.  This will introduce more lag into the system.
#install network tables in python by the following command:
#pip3 install --upgrade pynetworktables

from networktables import NetworkTables

NetworkTables.setIPAddress("169.254.18.93")
NetworkTables.setClientMode()
NetworkTables.initialize()
smartDashboard = NetworkTables.getTable('SmartDashboard')


fakeDistance = 1

print('Starting socket...')
HOST = ''
PORT = 50007

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.bind((HOST, PORT))

print('Listening on port', PORT,'...')
s.listen(1)

#Something connected, set up a connection
conn, addr = s.accept()
print('Port was connected to by', addr)
print('')

#After the intial connection, set a timeout (5 minute)
#to allow the socket to be non-blocking
s.settimeout(60)

while 1:
    #select.select will wait until data is available to be read on the socket.
    #Unlike recv (which also waits until data but has no timeout), select supports a timeout to
    #pseduo-create a non-blocking read.
    print('Waiting for activity on the connection...')    
    readable, writable, exceptional = select.select([conn], [], [conn], 60)
    print('Activity on the connection found')
    
    if readable:
        print('Reading data from the connection')
        data = conn.recv(1024)
    else:
        print('Some other port activity (possible exception)')
        break

    if not data:
        print('No data on the port.  Most likely a port disconnect')
        print('Returning to listening on the port for a connection')
        conn.close()

        #The listen/accept code should really be a function for code reuse
        s.listen(1)

        #Something connected, set up a connection
        conn, addr = s.accept()
        print('Port was connected to by', addr)
        print('')

        #the continue here is only valid for an actual connection.
        #if the timeout occurs, an exception is thrown and the app exits anyways
        continue

    #The data sent from the connection should be checked to see what it asking for (Distance) and/or distance off center.
    #We should also return the timestamp of the image along with the elapsed time since the image
    #was processed.  This would allow the auto code to use the processed image to see how well we
    #are moving towards the target.  We should be using the faster update sensors (such as the gyro
    #and encoders) to track our movement and using the camera image as a reference to see how well we
    #are doing.  We may get through the auto loop on the roborio several times between camera updates.
    
    print('Sending data back to the connection:', fakeDistance)
    conn.send(str(fakeDistance).encode())

    smartDashboard.putNumber('RaspPI Distance', fakeDistance)
    fakeDistance = fakeDistance + 1
    print('')

print('Closing connection and socket...')
conn.close()
s.shutdown(2)
s.close()
