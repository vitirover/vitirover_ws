# this python script uses python files generated by protobuf

# to install on windows, go to https://github.com/protocolbuffers/protobuf/releases and download for your architecture

# you can add it to your PATH using for example : 

# [Environment]::SetEnvironmentVariable('Path', $env:Path + ";C:\Users\{your user}\Downloads\protoc-24.4-win64\bin", [EnvironmentVariableTarget]::User)

# then, you can generate python files using

# protoc -I=protobuf/ --python_out=protobuf protobuf/telemetry.proto

import socket
import telemetry_pb2 as telemetry_pb2
import random
import time

from google.protobuf.text_format import MessageToString

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("192.168.2.106", 5005))
sock.setblocking(0)


while True:
    high_order = telemetry_pb2.VitiroverHighLevelOrder()
    high_order.speed = 50 + random.randint(0,50)
    # print(high_order.speed)
    high_order.back_axle_angle = 0
    high_order.turning_mode = telemetry_pb2.MANUAL

    order = telemetry_pb2.VitiroverOrder()

    order.high_level_order.CopyFrom(high_order)

    data = order.SerializeToString()

    try:
        sock.sendto(data, ("192.168.2.42", 5005))
        print("titi")
    except BlockingIOError:
        print("erreur on sendto")
        pass

    telemetry_data = None
    while True:
        try:
            data, addr = sock.recvfrom(20000)  # Buffer size
            telemetry_data = telemetry_pb2.VitiroverTelemetry()
        except BlockingIOError:
            # Plus de messages dans le buffer, sortir de la boucle
            break
    if telemetry_data != None:
        telemetry_data.ParseFromString(data)
        # Afficher quelques valeurs
        print(MessageToString(telemetry_data))


    time.sleep(0.5)


#    exit() 



