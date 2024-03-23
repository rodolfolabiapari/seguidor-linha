import socket
import sys
import function
import pickle

HOST = ""  # Symbolic name, meaning all available interfaces
PORT = 8888  # Arbitrary non-privileged port

ROBOT_SENSORS = "Master Robot"
ROBOT_WITHOUT_SENSORS = "Slave Robot"

main_command = "0"

def main():
    # Address Family : AF_INET (this is IP version 4 or IPv4)
    # Type : SOCK_STREAM (this means connection oriented TCP protocol)

    global main_command
    function.print_debug("[INFO] Starting the Ploudy", True)
    ploudy_server = function.start_server(HOST, PORT)

    l_data_base = []

    while not "x" in main_command:

        function.print_debug("[INFO] Waiting a new client... Size of List Map: " + str(len(l_data_base)), True)

        # wait to accept a connection - blocking call
        # addr0 is the IP and and addr1 is the ports
        client_connection, client_addr = ploudy_server.accept()
        function.print_debug("\n[INFO] Connected with " + client_addr[0] + " : " + str(client_addr[1]), True)

        robot = client_connection.recv(1024)

        function.print_debug("\n\tReceived: \"" + robot + "\"", True)

        print "(y) to accept\t (n) to recuse"
        main_command = sys.stdin.read(2)

        if ROBOT_SENSORS in robot and 'y' in main_command:
            main_command = '0'

            try:
                l_data_base = function.sensors_robot_procedure(client_connection)

                with open("master.map", 'wb') as f:
                    pickle.dump(l_data_base, f)
            except:
                function.print_debug("[ERROR] Client was desconnected by a error!", True)

        elif ROBOT_WITHOUT_SENSORS in robot and 'y' in main_command:

            print "(y) read from file"
            main_command = sys.stdin.read(2)

            if "y" in main_command:
                with open("master.map", 'rb') as f:
                    l_data_base = pickle.load(f)

            main_command = '0'
            if len(l_data_base) > 0:
                try:
                    function.slave_robot_procedure(client_connection, l_data_base)
                except:
                    function.print_debug("[ERROR] Client was desconnected by a error!", True)
            else:
                function.print_debug("[INFO] Client was desconnected by a error! \tEmpty map!", True)


        function.print_debug("[ERRO] Client was desconnected!", True)

        client_connection.close()

    ploudy_server.close()

main()