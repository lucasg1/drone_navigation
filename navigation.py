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

actualAng = 0
font=cv2.FONT_ITALIC
drone = Tello()
drone.connect()  
drone.streamon()
drone.set_speed(10)
keepRecording = True
frame_read = drone.get_frame_read()

position = np.float32([0,0]).reshape(-1,1,2)

def videoRecorder():
    # create a VideoWrite object, recoring to ./video.avi
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))

    while keepRecording:
        video.write(frame_read.frame)
        time.sleep(1 / 30)

    video.release()

def showVideo():
    while(1):
        img = frame_read.frame
        cv2.imshow("Drone video", img)

def goTo(drone, nextPos, actualPos,actualAng):
    distance = dist2D(nextPos, actualPos) # in meters
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

def goToSteps(drone, nextBasis, actualPos, steps):
    global actualAng
    distance = math.sqrt((nextBasis[0]-actualPos[0])**2+(nextBasis[1]-actualPos[1])**2) # distance from actualPos to nextBasis

    tangent = (nextBasis[1]-actualPos[1])/(nextBasis[0]-actualPos[0])
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

    if angleDif >= 0:
        drone.rotate_counter_clockwise(int(angleDif))
    else:
        drone.rotate_clockwise(int(-angleDif))

    actualAng = targetAng
    actualAng_radians = math.radians(actualAng)

    for j in range(0,steps):
        drone.move_forward(int(distance*100/steps))
        sleep(2)
        i = j + 1
        print('Position in ' ,i, 'step:', position)

        # After one step the drone should be in x_old + (distance/steps)*Cos(actualAngle)
        # deltaX = int(actualPos[0]+((i*distance*100)/steps)*math.cos(actualAng) - position[0][0][0])
        # deltaY = int(actualPos[0]+((i*distance*100)/steps)*math.sin(actualAng) - position[0][0][0])

        xPos_in_ith_step = actualPos[0]+((i*distance*100)/steps)*math.cos(actualAng_radians)
        yPos_in_ith_step = actualPos[1]+((i*distance*100)/steps)*math.sin(actualAng_radians)

        # vetor_actualPos = np.array([actualPos[0],actualPos[1]])
        # vetor_nextBasis = np.array([nextBasis[0],nextBasis[1]])
        # vetor_position = np.array([position[0][0][0], position[0][0][1]])
        # sides_correction = norm(np.cross(vetor_nextBasis-vetor_actualPos, vetor_actualPos-vetor_position))/norm(vetor_nextBasis-vetor_actualPos)
        # forward_correction = math.sqrt((xPos_in_ith_step-position[0][0][0])**2+(yPos_in_ith_step-position[0][0][1])**2 - sides_correction**2)

        small_to_original = np.float32([[math.cos(actualAng_radians), -math.sin(actualAng_radians), xPos_in_ith_step],[math.sin(actualAng_radians),math.cos(actualAng_radians),yPos_in_ith_step],[0,0,1]])
        transformation = np.linalg.inv(small_to_original)
        # transformation = np.float32([[math.cos(actualAng_radians), math.sin(actualAng_radians), ],[-math.sin(actualAng_radians),math.cos(actualAng_radians),0],[0,0,1]])
        posArray = np.float32([position[0][0][0],position[0][0][1],1])
        position_in_new_axis = np.matmul(transformation,posArray)



        if(int(position_in_new_axis[0]) < -19):
            drone.move_forward(abs(int(position_in_new_axis[0])))

        elif(int(position_in_new_axis[0]) > 19):
            drone.move_back(abs(int(position_in_new_axis[0])))

        if(int(position_in_new_axis[1]) < -19):
            drone.move_right(abs(int(position_in_new_axis[1])))

        elif(int(position_in_new_axis[1]) > 19):
            drone.move_left(abs(int(position_in_new_axis[1])))
def test(drone):
    actualAng = 0
    nextBasis = [0.8,0.8]
    actualPos = [0.0,0.0]

    distance = math.sqrt((nextBasis[0]-actualPos[0])**2+(nextBasis[1]-actualPos[1])**2) # distance from actualPos to nextBasis
    tangent = (nextBasis[1]-actualPos[1])/(nextBasis[0]-actualPos[0])
    targetAngle = math.degrees(math.atan(tangent))

    if nextBasis[1]-actualPos[1] == 0 and nextBasis[0] > actualPos[0]:
        targetAng = 0
    elif nextBasis[1]-actualPos[1] == 0 and nextBasis[0] < actualPos[0]:
        targetAng = 180
    elif nextBasis[0]-actualPos[0] == 0 and nextBasis[1] > actualPos[1]:
        targetAng = 90
    elif nextBasis[0]-actualPos[0] == 0 and nextBasis[1] < actualPos[1]:
        targetAng = -90

    angleDif = targetAng - actualAng
    print('angle dif: ', angleDif)
    print('target angle: ', targetAng)
    print('actual angle: ', actualAng)

    if angleDif >= 0:
        drone.rotate_counter_clockwise(int(angleDif))
    else:
        drone.rotate_clockwise(int(-angleDif))

    actualAng = targetAng


def mission(drone, steps):
    end_mission = True
    actualPos = [0,0,0,0] # x,y,z,theta
    searchHeight = 1.5
    i = 0

    for basis in map_constants.basis_coordinates:
        drone.takeoff()
        # drone.move_up(50)
        sleep(1)

        i = i + 1
        nextBasis = [basis[0],basis[1],searchHeight,0]

        print('going from {},{} to {},{}'.format(actualPos[0],actualPos[1],nextBasis[0],nextBasis[1]))
        # actualAng = goTo(drone, nextBasis, actualPos, actualAng)
        goToSteps(drone, nextBasis, actualPos, steps)

        drone.land()
        sleep(1)

        actualPos = nextBasis

def getPosition(capturador, estimador, currentHeight, result):
    while(1):
        global position
        startTime = time.time()
        old_position = position

        # frame_read.frame = capturador.getFrame()
        result, position = estimador.match(frame_read.frame, currentHeight)
        endTime = time.time()

        if(isinstance(position, type(None))):
            # print('Position is none. No match.')
            position = old_position
        elif(math.sqrt((position[0][0][0]-old_position[0][0][0])**2+(position[0][0][1]-old_position[0][0][1])**2) >= 30):
            position = old_position
            # print('The point is too far from the old point.')
        # else:
            # print('Position:', position)
        # print('FPS: ', 1/(endTime-startTime))

        # cv2.imshow('Matching',result)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

def main():
    currentHeight = 0.88
    steps = 5
    estimador = Estimador()
    capturador = Capturador()
    result = estimador.sceneMatching.templateImage
    # drawMap(map_constants.basis_coordinates, map_constants.arena_dimensions)
    
    recorder = threading.Thread(target=videoRecorder)
    recorder.start()
    # video = threading.Thread(target=showVideo, daemon=True)
    # video.start()

    vision = threading.Thread(target=getPosition, args=(capturador, estimador, currentHeight, result,), daemon=True)
    vision.start()

    navi = threading.Thread(target=mission, args=(drone, steps,))
    navi.start()
    
    # testing = threading.Thread(target=test, args=(drone,))
    # testing.start()

    while(1):
        resultRotated = cv2.rotate(result, cv2.cv2.ROTATE_90_COUNTERCLOCKWISE) 
        cv2.imshow('Navigation', resultRotated)
        img = frame_read.frame
        cv2.imshow("Drone video", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    keepRecording = False
    recorder.join()

if __name__ == '__main__':
    main()

    

