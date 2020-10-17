import time, cv2, math, constants
from time import sleep
from threading import Thread
from djitellopy import Tello
import numpy as np
font=cv2.FONT_ITALIC
drone = Tello()
drone.connect()  
drone.streamon()
drone.set_speed(10)
keepRecording = True
frame_read = drone.get_frame_read()

def videoRecorder():
    # create a VideoWrite object, recoring to ./video.avi
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))

    while keepRecording:
        video.write(frame_read.frame)
        time.sleep(1 / 30)

    video.release()

def showVideo():
    img = frame_read.frame
    cv2.imshow("Drone video", img)

def drawMap(basis_coordinates, arena_dimensions):
    img = np.zeros((constants.image_size[0],constants.image_size[1],3), np.uint8)

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
    x = int((coord[0]/constants.arena_dimensions[0])*constants.image_size[0])
    y = int((coord[1]/constants.arena_dimensions[1])*constants.image_size[1])
    y = 512 - y

    return x, y

def goTo(drone, nextPos, actualPos,actualAng):
    distance = math.sqrt((nextPos[0]-actualPos[0])**2+(nextPos[1]-actualPos[1])**2) # in meters
    print('distance is ' + str(distance))

    if nextPos[0] == actualPos[0]:
        angle = 270
    else:
        tangent = (nextPos[1]-actualPos[1])/(nextPos[0]-actualPos[0]) # rotate clockwise by ArcTangent(tangent)
        angle = math.degrees(math.atan(tangent))
    
    if angle < 0:
        angle = 180 - math.fabs(angle)
    resp = angle - actualAng

    print('resp is ' + str(resp))
    print('angle is ' + str(angle))
    print('actualAng is ' + str(actualAng))

    # if resp < 0:
    #     drone.rotate_clockwise(int(-resp))
    # else:
    #     drone.rotate_counter_clockwise(int(resp))

    drone.move_forward(int(distance*100)) # in cm

    return angle

def mission(drone):
    actualAng = 90
    actualPos = [0,0,0,0] # x,y,z,theta
    searchHeight = 1.5
    i = 0

    for basis in constants.basis_coordinates:
        i = i + 1
        nextPos = [basis[0],basis[1],searchHeight,0]

        print('going from {},{} to {},{}'.format(actualPos[0],actualPos[1],nextPos[0],nextPos[1]))
        actualAng = goTo(drone, nextPos, actualPos, actualAng)


        drone.land()
        sleep(2)
        return
        actualPos = nextPos


        drone.takeoff()
        drone.move_up(50)

    nextPos = [0,0,searchHeight,0]
    actualAng = goTo(drone,nextPos,actualPos,actualAng)
   
    drone.land()

def main():
    
    # drawMap(constants.basis_coordinates, constants.arena_dimensions)
    

    recorder = Thread(target=videoRecorder)

    recorder.start()

    video = Thread(target=showVideo)

    video.start()


    drone.takeoff()
    drone.move_up(50)
    sleep(5)
    drone.land()
    # mission(drone)

    keepRecording = False
    recorder.join()

if __name__ == '__main__':
    main()

    

