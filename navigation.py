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

time_LastValidPosition = time.time()
actualAng = 0
font=cv2.FONT_ITALIC
drone = Tello()
drone.connect()  
drone.streamon()
drone.set_speed(10)
keepRecording = True
frame_read = drone.get_frame_read()
stopCode = False

position = np.float32([0,0]).reshape(-1,1,2)
notNonePosition = np.float32([0,0]).reshape(-1,1,2)

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

def checkOrientationCornerCases(actualPos, nextBasis, targetAngOriginal):
    targetAng = 0

    if nextBasis[1]-actualPos[1] == 0 and nextBasis[0] > actualPos[0]:
        targetAng = 0
    elif nextBasis[1]-actualPos[1] == 0 and nextBasis[0] < actualPos[0]:
        targetAng = -180
    elif nextBasis[0]-actualPos[0] == 0 and nextBasis[1] > actualPos[1]:
        targetAng = 90
    elif nextBasis[0]-actualPos[0] == 0 and nextBasis[1] < actualPos[1]:
        targetAng = -90
    else:
        return targetAngOriginal

def logAngles(actualPos, nextBasis, angleDif, targetAng, actualAng):
    print('actualPos: ',actualPos)
    print('nextBasis: ',nextBasis)
    print('angle dif: ', angleDif)
    print('target angle: ', targetAng)
    print('actual angle: ', actualAng)

def calcAveragePositions(array):
    xSum = 0
    ySum = 0
    for pos in array:
        xSum = xSum+pos[0][0][0]
        ySum = ySum+pos[0][0][1]

    xAverage = xSum/len(array)
    yAverage = ySum/len(array)

    return [xAverage, yAverage]

def goToSteps(drone, nextBasis, actualPos, steps):
    global actualAng
    distance = math.sqrt((nextBasis[0]-actualPos[0])**2+(nextBasis[1]-actualPos[1])**2) # distance from actualPos to nextBasis

    tangent = (nextBasis[1]-actualPos[1])/(nextBasis[0]-actualPos[0])
    targetAng = math.degrees(math.atan(tangent))
    
    targetAng = checkOrientationCornerCases(actualPos, nextBasis, targetAng)

    angleDif = targetAng - actualAng
    
    logAngles(actualPos, nextBasis, angleDif, targetAng, actualAng)

    if angleDif >= 0:
        drone.rotate_counter_clockwise(int(angleDif))
    else:
        drone.rotate_clockwise(int(-angleDif))

    actualAng = targetAng
    actualAng_radians = math.radians(actualAng)

    for j in range(0,steps):
        if(stopCode == True): # check if someone is stopping the code
            return

        drone.move_forward(int(distance*100/steps))

        beginObtainingPositions = time.time()
        estimatePositionArray = []
        estimatePositionArray.append(position)
        while(time.time()-beginObtainingPositions <= 1):
            if(estimatePositionArray[len(estimatePositionArray)-1][0][0][0] != position[0][0][0] and estimatePositionArray[len(estimatePositionArray)-1][0][0][1] != position[0][0][1]):
                estimatePositionArray.append(position)
                print('Estimated position: ', calcAveragePositions(estimatePositionArray))

        i = j + 1
        print('Position in ' ,i, 'step:', position)

        # After one step the drone should be in x_old + (distance/steps)*Cos(actualAngle)
        # deltaX = int(actualPos[0]+((i*distance*100)/steps)*math.cos(actualAng) - position[0][0][0])
        # deltaY = int(actualPos[0]+((i*distance*100)/steps)*math.sin(actualAng) - position[0][0][0])

        
        print('actualPos: ', actualPos)
        print('distance: ', distance*100)
        print('cos: ', math.cos(actualAng_radians))
        print('sin: ', math.sin(actualAng_radians))
        xPos_in_ith_step = actualPos[0]*100+((i*distance*100)/steps)*math.cos(actualAng_radians)
        yPos_in_ith_step = actualPos[1]*100+((i*distance*100)/steps)*math.sin(actualAng_radians)
        print('The drone should be moving to: (', xPos_in_ith_step, ', ', yPos_in_ith_step, ')')

        # vetor_actualPos = np.array([actualPos[0],actualPos[1]])
        # vetor_nextBasis = np.array([nextBasis[0],nextBasis[1]])
        # vetor_position = np.array([position[0][0][0], position[0][0][1]])
        # sides_correction = norm(np.cross(vetor_nextBasis-vetor_actualPos, vetor_actualPos-vetor_position))/norm(vetor_nextBasis-vetor_actualPos)
        # forward_correction = math.sqrt((xPos_in_ith_step-position[0][0][0])**2+(yPos_in_ith_step-position[0][0][1])**2 - sides_correction**2)

        small_to_original = np.float32([[math.cos(actualAng_radians), -math.sin(actualAng_radians), xPos_in_ith_step],[math.sin(actualAng_radians),math.cos(actualAng_radians),yPos_in_ith_step],[0,0,1]])
        transformation = np.linalg.inv(small_to_original)
        posArray = np.float32([position[0][0][0],position[0][0][1],1])
        position_in_new_axis = np.matmul(transformation,posArray)
        print('small_to_original: ', small_to_original)

        print('transformation: ', transformation)
        print('position in new axis: ', position_in_new_axis)

        if(int(position_in_new_axis[0]) < -19):
            drone.move_forward(abs(int(position_in_new_axis[0])))

        elif(int(position_in_new_axis[0]) > 19):
            drone.move_back(abs(int(position_in_new_axis[0])))

        if(int(position_in_new_axis[1]) < -19):
            drone.move_right(abs(int(position_in_new_axis[1])))

        elif(int(position_in_new_axis[1]) > 19):
            drone.move_left(abs(int(position_in_new_axis[1])))

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
        goToSteps(drone, nextBasis, actualPos, steps)

        drone.land()
        sleep(1)

        actualPos = nextBasis

def getPosition(capturador, estimador, currentHeight, result):
    printTimer = time.time()

    while(not stopCode):
        global position, notNonePosition, time_LastValidPosition


        # frame_read.frame = capturador.getFrame()
        result, positionFromVision = estimador.match(frame_read.frame, currentHeight)

        if(not (isinstance(positionFromVision, type(None)) or math.sqrt((position[0][0][0]-positionFromVision[0][0][0])**2+(position[0][0][1]-positionFromVision[0][0][1])**2) >= 50)):
            position = positionFromVision
            time_LastValidPosition = time.time()

        if(not (isinstance(positionFromVision, type(None)))):
            notNonePosition = positionFromVision
            time_LastValidPosition = time.time()

        timeSinceLastValidPosition = time.time() - time_LastValidPosition

        if(timeSinceLastValidPosition > 4):
            position = notNonePosition
            time_LastValidPosition = time.time()

        if(time.time()-printTimer > 1):
            printTimer = time.time()
            print('Time since last valid position: ', timeSinceLastValidPosition)
            print('Last valid position: ', position)

            # print('Position is none. No match.')
            # The point is too far from the old point.

            # print('The point is too far from the old point.')
        # else:
            # print('Position:', position)
        # print('FPS: ', 1/(endTime-startTime))

        # cv2.imshow('Matching',result)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

def main():
    currentHeight = 0.88
    steps = 1
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
        global stopCode
        resultRotated = cv2.rotate(result, cv2.cv2.ROTATE_90_COUNTERCLOCKWISE) 
        cv2.imshow('Navigation', resultRotated)
        img = frame_read.frame
        cv2.imshow("Drone video", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('Stopping everything and landing the drone!!!')
            stopCode = True
            navi.join()
            vision.join()
            drone.land()
            keepRecording = False
            recorder.join()
    
if __name__ == '__main__':
    main()

    

