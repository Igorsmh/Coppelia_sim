import sim
from time import sleep as delay
import sys

print('Program started')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

lSpeed = 0.0
rSpeed = 0.0

if (clientID != -1):
    print('Connected to remote API server')
else:
    sys.exit('Failed connecting to remote API server')

delay(1)

# Obtenha o identificador do sensor de proximidade
errorCode, proximity_sensor_handle = sim.simxGetObjectHandle(
    clientID, '/PioneerP3DX/ultrasonicSensor', sim.simx_opmode_oneshot_wait)

error_code, left_motor_handle = sim.simxGetObjectHandle(clientID,
                                                        '/PioneerP3DX/leftMotor', sim.simx_opmode_oneshot_wait)
error_code, right_motor_handle = sim.simxGetObjectHandle(clientID,
                                                         '/PioneerP3DX/rightMotor', sim.simx_opmode_oneshot_wait)

if errorCode == 0:
    print('Proximity sensor handle obtained')
else:
    sys.exit('Failed to get proximity sensor handle')

try:
    while clientID != -1:
        # Leia os dados do sensor de proximidade
        errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            clientID, proximity_sensor_handle, sim.simx_opmode_streaming)

        if errorCode == 0:
            print(detectedObjectHandle)
            if detectionState:
                print('Object detected')
                lSpeed = 0.4  # Pare o motor esquerdo
                rSpeed = 0.2  # Pare o motor direito
            else:
                print('No object detected')
                lSpeed = 0.2  # Velocidade do motor esquerdo
                rSpeed = 0.2  # Velocidade do motor direito

            # Ajuste as velocidades dos motores
            sim.simxSetJointTargetVelocity(
                clientID, left_motor_handle, lSpeed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, right_motor_handle, rSpeed, sim.simx_opmode_oneshot)

        delay(0.1)


except KeyboardInterrupt:
    # Lidar com a interrupção do teclado (Ctrl+C)
    print('KeyboardInterrupt: Stopping simulation...')
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
    sys.exit()
