from __future__ import print_function
import socket
import sys
import time
import function
import pickle

HOST = ""  # Symbolic name, meaning all available interfaces
PORT = 8888  # Arbitrary non-privileged port

ROBOT_SENSORS = "Master Robot"
ROBOT_WITHOUT_SENSORS = "Slave Robot"

main_command = "0"
import threading

command = "0"

class Command_thread ( threading.Thread ):

    def run ( self ):
        print("Command Thread Started")

        global command

        print("(d) done\t(r) restart the robot")
        command = sys.stdin.read(2)

        print("Command Thread Finnished")


DEBUG = True

max_rotation_motor = 1023
Kp = 35
reference = 2

def print_debug(s, new_line):
    global DEBUG

    if DEBUG:
        if new_line:
            print(s)
        else:
            print(s, end="")

def start_server(HOST, PORT):
    try:
        # create an AF_INET, STREAM socket (TCP)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except socket.error, msg:
        print("[INFO] Failed to create socket. Error code: " + str(msg[0]) + " , Error message : " + msg[1])
        sys.exit()

    print("[INFO] Socket created")

    # Bind socket to local host and port
    try:
        s.bind((HOST, PORT))
    except socket.error as msg:
        print("[INFO] Bind failed. Error Code : " + str(msg[0]) + " Message " + msg[1])
        sys.exit()

    print("[INFO] Socket bind complete")

    # Start listening on socket for ten clients
    s.listen(10)
    print("[INFO] Socket now listening")

    return s

def verify_parameters (client):
    package = client.recv(1024)

    list_package = package.split()

    if len(list_package) == 5:
        time = int(list_package[0])

        led_esq = int(list_package[1])
        led_dir = int(list_package[2])

        left_motor = float(list_package[3])
        righ_motor = float(list_package[4])

    else:
        print_debug("[ERRO] Erro na leitura do sensor", True)
        return 0, 0, 0, 0, 0, 0


    print_debug("tme:" + '{:07d}'.format(time), False)
    print_debug("  lig:" + str(led_esq) + ":" + str(led_dir), False)
    print_debug("  enc:" + '{:04d}'.format(int(left_motor)) + ":" + '{:04d}'.format(int(righ_motor)), False)

    return time, led_esq, led_dir, left_motor, righ_motor

def send_new_rotations(client, left_rotation_motor, righ_rotation_motor):
    new_package = str(left_rotation_motor) + " " + str(righ_rotation_motor)

    client.sendall(new_package)

def send_command(client, s_string):
    client.sendall(s_string)

def calcule_proportion(current_encoder, reference):
    position = current_encoder
    perfect_position = reference

    proportion = perfect_position - position

    error_value = proportion * Kp

    return error_value

def calcule_spin(left_rotation_motor, righ_rotation_motor, left_error_value, righ_error_value, left_light, righ_light):
    min = 0 
    max = max_rotation_motor

    if left_light == 0:
        left_rotation_motor += float((left_error_value * abs(max_rotation_motor - left_rotation_motor)) / 100)
    else:
        left_rotation_motor += -max

    if righ_light == 0:
        righ_rotation_motor += float((righ_error_value * abs(max_rotation_motor - righ_rotation_motor)) / 100)
    else:
        righ_rotation_motor += -max


    if left_rotation_motor > max:
        left_rotation_motor = max
    elif left_rotation_motor < min:
        left_rotation_motor = min

    if righ_rotation_motor > max:
        righ_rotation_motor = max
    elif righ_rotation_motor < min:
        righ_rotation_motor = min

    print_debug("     PWM(L:R):" + '{:04d}'.format(int(left_rotation_motor)) + "  " + '{:04d}'.format(int(righ_rotation_motor)), True)

    return int(left_rotation_motor), int(righ_rotation_motor)

def sensors_robot_procedure(client_connection):
    print_debug("[INFO] <time>  <left_light> <light_ righ>  <left_motor> <right_motor>", True)

    global command

    command = "r"
    l_data_base = []

    while "r" in command:
        print("(y) to start the robot")
        command = sys.stdin.read(2)

        send_command(client_connection, "s")

        sys.stdin.flush()

        Command_thread().start()

        l_data_base = []
        last_left_rotation_motor = 0
        last_righ_rotation_motor = 0
        left_rotation_motor = 0
        righ_rotation_motor = 0
        left_sum_sensor = 0
        righ_sum_sensor = 0
        time = 0

        command = "a"

        # now keep talking with the client
        while "a" in command:
            # Verify the parameters
            time, left_light, righ_light, current_left_motor_frames, current_righ_motor_frames = verify_parameters(client_connection)


            factor_left_rotation_motor = float(current_left_motor_frames - last_left_rotation_motor)
            factor_righ_rotation_motor = float(current_righ_motor_frames - last_righ_rotation_motor)

            if factor_left_rotation_motor > 12:
                factor_left_rotation_motor = 0


            if factor_righ_rotation_motor > 12:
                factor_righ_rotation_motor = 0

            last_left_rotation_motor = current_left_motor_frames
            last_righ_rotation_motor = current_righ_motor_frames

            print_debug("   |   Dif_btwn_Encd:" + str(int(factor_left_rotation_motor)) + " " + str(int(factor_righ_rotation_motor)), False)


            if left_light == 0:
                left_error_value = calcule_proportion(factor_left_rotation_motor, reference)
            else:
                left_error_value = calcule_proportion(factor_left_rotation_motor, 0)

            if righ_light == 0:
                righ_error_value = calcule_proportion(factor_righ_rotation_motor, reference)
            else:
                righ_error_value = calcule_proportion(factor_righ_rotation_motor, 0)


            print_debug("     ERROR(L:R):" + '{: 03d}'.format(int(left_error_value)) + "  " + '{: 03d}'.format(int(righ_error_value)), False)

            left_rotation_motor, righ_rotation_motor = calcule_spin(left_rotation_motor,
                                                                    righ_rotation_motor, left_error_value, righ_error_value, left_light, righ_light)
            l_data_base.append([time, left_rotation_motor, righ_rotation_motor,  current_left_motor_frames, current_righ_motor_frames])


            send_new_rotations(client_connection, left_rotation_motor, righ_rotation_motor)

        print_debug("[INFO] Command pressed: " + command, True)

        send_new_rotations(client_connection, 0, 0)
        l_data_base.append([time + 50, 0, 0,  0, 0])

    command = "0"

    return l_data_base

# =======================================================================================

def send_new_rotations_slave(client, list_rotations):
    reference = 0
    client.sendall(str(len(list_rotations)) + "\n")

    for i in range(0, len(list_rotations) - 1):
        time = list_rotations[i][0]

        if list_rotations[i][1] < max_rotation_motor:
            left_motor = int(list_rotations[i][1])
        else:
            left_motor = list_rotations[i][1]

        if list_rotations[i][2] < max_rotation_motor:
            righ_motor = int(list_rotations[i][2])
        else:
            righ_motor = list_rotations[i][2]

        left_motor_frames = list_rotations[i][3]
        righ_motor_frames = list_rotations[i][4]

        client.sendall(str(int(time)) + "  " + str(int(left_motor)) + " " + str(int(righ_motor)) + "  " +
                       str(int(left_motor_frames)) + " " + str(int(righ_motor_frames)) + "\n\0")

        client.recv(128)


def slave_robot_procedure(client_connection, l_data_base):
    print_debug("[INFO] <time>  <left_motor> <right_motor>", True)

    global command
    print("(y) to start the robot")
    command = sys.stdin.read(2)

    send_command(client_connection, "s")

    sys.stdin.flush()
    command = '0'

    Command_thread().start()

    print_debug("[INFO] Sending the Map", True)
    send_new_rotations_slave(client_connection, l_data_base)

    print_debug("[INFO] Sending Done", True)

    print_debug("[INFO] Closing Connection", True)


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