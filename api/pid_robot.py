import sim
from time import sleep as delay
import sys


def calculate_pid_error(previous_error, error, integral, Kp, Ki, Kd):
    """
    Calculate the proportional, integral, and derivative terms for a PID controller.

    Parameters:
        previous_error (float): The previous error value.
        error (float): The current error value.
        integral (float): The accumulated integral value.
        Kp (float): The proportional gain.
        Ki (float): The integral gain.
        Kd (float): The derivative gain.

    Returns:
        tuple: A tuple containing the proportional, integral, and derivative terms (P, I, D) and the updated integral value.

    """
    # Cálculo do termo proporcional
    P = Kp * error

    # Cálculo do termo integral
    integral += error
    I = Ki * integral

    # Cálculo do termo derivativo
    D = Kd * (error - previous_error)

    return P, I, D, integral


print('Program started')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print('Connected to remote API server')
else:
    sys.exit('Failed connecting to remote API server')

delay(1)

# Obtenha os identificadores dos motores esquerdo e direito
errorCode, left_motor_handle = sim.simxGetObjectHandle(
    clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = sim.simxGetObjectHandle(
    clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_oneshot_wait)

if errorCode == 0:
    print('Motor handle obtained')
else:
    sys.exit('Failed to get motor handle')

delay(1)

# Obtenha o identificador do sensor de proximidade
errorCode, proximity_sensor_handle = sim.simxGetObjectHandle(
    clientID, '/PioneerP3DX/ultrasonicSensor', sim.simx_opmode_oneshot_wait)

if errorCode == 0:
    print('Proximity sensor handle obtained')
else:
    sys.exit('Failed to get proximity sensor handle')

delay(1)

# Parâmetros do controlador PID
Kp = 0.2  # Ganho proporcional
Ki = 0.001  # Ganho integral
Kd = 0.01  # Ganho derivativo

integral = 0.0
previous_error = 0.0
error = 0.0

try:
    while True:
        # Leia os dados do sensor de proximidade
        errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            clientID, proximity_sensor_handle, sim.simx_opmode_streaming)

        if errorCode == 0:
            if detectionState:
                # Objeto detectado, calcular o erro
                error = 0.2 - detectedPoint[2]  # 0.1 é a distância desejada

                # Calcule os termos PID
                P, I, D, integral = calculate_pid_error(
                    previous_error, error, integral, Kp, Ki, Kd)

                # Ajuste as velocidades dos motores com base nos termos PID
                left_speed = 0.2 - P - I - D
                right_speed = 0.2 + P + I + D
            else:
                # Nenhum objeto detectado, continue para a frente
                left_speed = 0.2
                right_speed = 0.2

            # Ajuste as velocidades dos motores
            sim.simxSetJointTargetVelocity(
                clientID, left_motor_handle, left_speed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, right_motor_handle, right_speed, sim.simx_opmode_oneshot)

            # Atualize o erro anterior
            previous_error = error

except KeyboardInterrupt:
    # Lidar com a interrupção do teclado (Ctrl+C)
    print('KeyboardInterrupt: Stopping simulation...')
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
    sys.exit()
