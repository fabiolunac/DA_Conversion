import serial
import numpy as np
import matplotlib.pyplot as plt

####################### Configurar porta serial #######################

portal_serial = serial.Serial('COM5', baudrate=115200, timeout=1)


####################### Enviar vetor para o Tiva #######################

def send_int_vector(serialobj: serial.Serial, vector: np.array) -> None:
    for value in vector:
        serialobj.write(np.float32(value).tobytes())
    
####################### Gerar sinal #######################

f = 200
Fs = 10000
Nc = 2
N = int(Fs/f * Nc)

t = np.linspace(0, N, 100)

x = np.sin(2*np.pi*t*f/Fs)

# plt.figure()
# plt.plot(t,x)
# plt.show()

####################### Enviar o vetor para o Tiva #######################

send_int_vector(portal_serial, x)
