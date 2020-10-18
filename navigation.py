import time, cv2, math, map_constants
from time import sleep
from threading import Thread
from djitellopy import Tello
from capturador import Capturador
from estimador import Estimador
from constants import isSingleTest
import threading
import numpy as np

font=cv2.FONT_ITALIC
drone = Tello()
drone.connect()  
drone.streamon()
drone.set_speed(10)
keepRecording = True
frame_read = drone.get_frame_read()

def dist2D(a, b):
    distance = math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2) # in meters
    return distance

def videoRecorder():
    # create a VideoWrite object, recoring to ./video.avi
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))

    while keepRecording:
        video.write(frame_read.frame)
        time.sleep(1 / 30)

    video.release()

def drawMap(basis_coordinates, arena_dimensions):
    img = np.zeros((map_constants.image_size[0],map_constants.image_size[1],3), np.uint8)

    img = cv2.rectangle(img,(-10,-10),(10,10),(0,255,0),3)

    basis_number = 0

    for coord in basis_coordinates:
        basis_number = basis_number + 1
        x, y = convertFromRealToImage(coord)
        print('coord é ' + str(coord[0])+ ' ' + str(coord[1]))
        print('x e y sao' + str(x) + ' ' + str(y))

        img = cv2.rectangle(img,(x-10,y+10),(x+10,y-10),(0,255,0),3)
        basisName = 'base ' + str(basis_number)
        img = cv2.putText(img, basisName, (x-10,y+10), font, 1, (255,255,255), 1, cv2.LINE_AA)
    # img = cv2.flip(img,0) # rotaciona a imagem verticalmente para o mapa estar no primeiro quadrante

    cv2.imshow('map',img)
    cv2.waitKey(0)
    cv2.destroyWindow('map')

def convertFromRealToImage(coord):
    x = int((coord[0]/map_constants.arena_dimensions[0])*map_constants.image_size[0])
    y = int((coord[1]/map_constants.arena_dimensions[1])*map_constants.image_size[1])
    y = 512 - y

    return x, y

def goTo(drone, nextPos, actualPos,actualAng):
    distance = Dist2D((nextPos, actualPos)) # in meters
    # print('distance is ' + str(distance))

    if nextPos[0] == actualPos[0]:
        angle = 270
    else:
        tangent = (nextPos[1]-actualPos[1])/(nextPos[0]-actualPos[0]) # rotate clockwise by ArcTangent(tangent)
        angle = math.degrees(math.atan(tangent))
    
    if angle < 0:
        angle = 180 - math.fabs(angle)
    resp = angle - actualAng

    # print('resp is ' + str(resp))
    # print('angle is ' + str(angle))
    # print('actualAng is ' + str(actualAng))

    # if resp < 0:
    #     drone.rotate_clockwise(int(-resp))
    # else:
    #     drone.rotate_counter_clockwise(int(resp))

    drone.move_forward(int(distance*100)) # in cm

    return angle

def goToSteps(drone, nextPos, actualPos, actualAng, steps, position):
    distance = Dist2D((nextPos, actualPos)) # in meters
    for i in steps:
        print('Position in ' ,i, 'step:', position)
        drone.move_forward(int(distance*100/steps))

def mission(drone, steps, position):
    end_mission = True
    while(end_mission):

        actualAng = 90
        actualPos = [0,0,0,0] # x,y,z,theta
        searchHeight = 1.5
        i = 0

        drone.takeoff()
        drone.move_up(50)

        for basis in map_constants.basis_coordinates:
            i = i + 1
            nextBasis = [basis[0],basis[1],searchHeight,0]

            print('going from {},{} to {},{}'.format(actualPos[0],actualPos[1],nextBasis[0],nextBasis[1]))
            # actualAng = goTo(drone, nextBasis, actualPos, actualAng)
            actualAng = goToSteps(drone, nextBasis, actualPos, actualAng, steps, position)


            drone.land()
            sleep(2)

            return
            actualPos = nextBasis


            drone.takeoff()
            drone.move_up(50)

        nextPos = [0,0,searchHeight,0]
        actualAng = goTo(drone,nextBasis,actualPos,actualAng)
   
        drone.land()
        end_mission = False

def getPosition(capturador, estimador, position, currentHeight):
    while(1):
        startTime = time.time()
        old_position = position

        # frame_read.frame = capturador.getFrame()
        result, position = estimador.match(frame_read.frame, currentHeight)
        endTime = time.time()

        if(dist2D(position, old_position) >= 0.05):
            position = old_position
            print('The point is too far from the old point.')

        print('FPS: ', 1/(endTime-startTime))
        print('Position:', position)

def showVideo():
    while(1):
        img = frame_read.frame
        cv2.imshow("Drone video", img)

def main():
    currentHeight = 1.28
    steps = 5
    position = [0,0]
    estimador = Estimador()
    capturador = Capturador()
    # drawMap(map_constants.basis_coordinates, map_constants.arena_dimensions)
    
    recorder = threading.Thread(target=videoRecorder)
    recorder.start()
    # video = threading.Thread(target=showVideo, daemon=True)
    # video.start()

    vision = threading.Thread(target=getPosition, args=(capturador, estimador, position, currentHeight,), daemon=True)
    vision.start()

    mission(drone, steps, position)

    keepRecording = False
    recorder.join()

if __name__ == '__main__':
    main()

    

