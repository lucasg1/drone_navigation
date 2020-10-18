import time, cv2, math, map_constants
from time import sleep
from threading import Thread
from djitellopy import Tello
from capturador import Capturador
from estimador import Estimador
from constants import isSingleTest
import numpy as np
import threading

def outra(a):
    while(1):
        print('Na funcao outra: ' + str(a))
        sleep(1)

def sum(a):
    while(1):
        # print(a)
        a = a + 1
        sleep(1)
def main():
    a = 1
    teste = threading.Thread(target=sum, args=(a,))
    teste.start()
    outra(a)
if __name__ == '__main__':
    main()

    

