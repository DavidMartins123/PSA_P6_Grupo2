from collections import namedtuple
import serial
import math
import time
import threading

import elite_json as ejson 
import os

# Configuração da comunicação serial
ser = serial.Serial('COM4', 115200)
ser.flushInput()  # descarta qualquer dado pendente no buffer de entrada

Accelerometer = namedtuple('Accelerometer', ['x', 'y', 'z'])
Gyroscope = namedtuple('Gyroscope', ['x', 'y', 'z'])
a = Accelerometer(0, 0, 0)
g = Gyroscope(0, 0, 0)

ultima_leitura = ""
def receiving(ser):
    global ultima_leitura
    buffer = ""

    while True:
        buffer += ser.read(ser.inWaiting()).decode('iso-8859-1')
        if "\n" in buffer:
            ultima_leitura, buffer = buffer.split("\n")[-2:]

            
def main():
    # loop infinito para leitura de dados
    x_global = 0
    y_global = 0
    z_global = 0

    rx = 0
    ry = 0
    rz = 0

    ip = "192.168.1.66"
    conSuc, sock = ejson.connectETController(ip)


    if conSuc:   #só executa o código se conectar ao robot
        
        # transparent transmission init
        suc, result, id = ejson.sendCMD(sock, "transparent_transmission_init",
                                      {"lookahead": 200, "t": 80, "smoothness": 1})

        while True:
            ret , pos_inicial , id = ejson.sendCMD(sock,"get_base_flange_pose",{"unit_type":1})
            print("Posição inicial: ", pos_inicial)
 

            # espera antes de ler a porta serial
            time.sleep(0.01)

            data = ultima_leitura

            # processa os dados recebidos
            if 1:
                try:
                    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = [float(x) for x in data.split(",")]
                    print(f"accel_x={accel_x}, accel_y={accel_y}, accel_z={accel_z}, gyro_x={gyro_x}, gyro_y={gyro_y}, gyro_z={gyro_z}")

                    x = accel_x*20
                    y = accel_y*20
                    z = accel_z*20

                    # conversão para radianos
                    rx = gyro_x*3.14/180
                    ry = gyro_y*3.14/180
                    rz = gyro_z*3.14/180

                    x_global = x
                    y_global = y
                    z_global = z


                    # Impressão das coordenadas espaciais
                    print("X: ", x_global)
                    print("Y: ", y_global)
                    print("Z: ", z_global)                                   
        
                    posicao = [x_global, y_global, z_global, gyro_x, gyro_y, gyro_z]

                    posicao[0] = pos_inicial[0] - x
                    posicao[1] = pos_inicial[1] - y
                    posicao[2] = pos_inicial[2] + z
                    posicao[3] = pos_inicial[3] + ry
                    posicao[4] = pos_inicial[4] - rx
                    posicao[5] = pos_inicial[5] + rz
                    print("Posição seguinte XYZ: ", posicao)

                    suc, p_target, id = ejson.sendCMD(sock, "inverseKinematic", {"targetPose": posicao})
                    print("Posição seguinte Joint: ", p_target)

                    suc, result, id = ejson.sendCMD(sock, "tt_put_servo_joint_to_buf", {"targetPos": p_target})
                    
                    # Get the servo status of the robotic arm
                    suc, result , id=ejson.sendCMD(sock, "getServoStatus")
                    if  result == 0:
                        #   Set the servo status of the robot arm to ON
                        suc, result , id=ejson.sendCMD(sock,"set_servo_status",{" status ":1})
                        time.sleep(1)

                except ValueError:
                    print("Invalid data:", data)
                    # Atribui valores padrão caso ocorra um erro na conversão dos dados
                    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = 0, 0, 0, 0, 0, 0
                    a = Accelerometer(accel_x, accel_y, accel_z)
                    g = Gyroscope(gyro_x, gyro_y, gyro_z) 


thread_ultimo = threading.Thread(target=receiving, args=(ser,), daemon=True)

thread_ultimo.start()
main()
