import time, cv2, math, map_constants
from time import sleep
from threading import Thread
from djitellopy import Tello
from capturador import Capturador
from estimador import Estimador
from constants import isSingleTest
from mapUtil import drawMap, convertFromRealToImage
import threading
import numpy as np

def test():
    actualAng = 45
    actualPos = [0.8,0.8]
    nextBasis = [1.5,0]

    distance = math.sqrt((nextBasis[0]-actualPos[0])**2+(nextBasis[1]-actualPos[1])**2) # distance from actualPos to nextBasis
    tangent = (nextBasis[1]-actualPos[1])/(nextBasis[0]-actualPos[0])
    print('tangent: ', tangent)
    targetAng = math.degrees(math.atan(tangent))

    if nextBasis[1]-actualPos[1] == 0 and nextBasis[0] > actualPos[0]:
        targetAng = 0
    elif nextBasis[1]-actualPos[1] == 0 and nextBasis[0] < actualPos[0]:
        targetAng = -180
    elif nextBasis[0]-actualPos[0] == 0 and nextBasis[1] > actualPos[1]:
        targetAng = 90
    elif nextBasis[0]-actualPos[0] == 0 and nextBasis[1] < actualPos[1]:
        targetAng = -90

    angleDif = targetAng - actualAng
    print('actualPos: ',actualPos)
    print('nextBasis: ',nextBasis)
    print('angle dif: ', angleDif)
    print('target angle: ', targetAng)
    print('actual angle: ', actualAng)

    # if angleDif >= 0:
    #     drone.rotate_counter_clockwise(int(angleDif))
    # else:
    #     drone.rotate_clockwise(int(-angleDif))

    # actualAng = targetAng
    #xPos_in_ith_step = 1
    #yPos_in_ith_step = 1
    #actualAng_radians = math.radians(actualAng)
    #small_to_original = np.float32([[math.cos(actualAng_radians), -math.sin(actualAng_radians), xPos_in_ith_step],[math.sin(actualAng_radians),math.cos(actualAng_radians),yPos_in_ith_step],[0,0,1]])
    #transformation = np.linalg.inv(small_to_original)
    # transformation = np.float32([[math.cos(math.radians(actualAng)),math.sin(math.radians(actualAng)),-math.sqrt(2)],[-math.sin(math.radians(actualAng)),math.cos(math.radians(actualAng)),0],[0,0,1]])
    # transformation = np.linalg.inv(transformation)
    #print(transformation)
    #posArray = np.float32([1,1+math.sqrt(2),1])
    # position_in_new_axis = np.matmul(transformation,posArray)

    # print('transformation result: ', position_in_new_axis)

if __name__ == '__main__':
    test()