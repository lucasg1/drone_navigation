import numpy as np

from enums import ImageInput

# MAIN CONSTANTS
isSingleTest = False

# CAPTURADOR CONSTANTS
inputType = ImageInput.VIDEO
videoFile = 'imagensTeste/teste4.mp4'

# ESTIMADOR CONSTANTS
templateImage = 'imagensTeste/experimento3Referencial25.jpg'
singleImageTest = 'golden.jpeg'
mapTemplatePoints = np.float32([[ 161 , 550 ],[ 32 , 434 ],[ 194 , 426 ],[ 997 , 387 ],[ 862 , 177 ]]).reshape(-1,1,2)
mapRealPoints = np.float32([[0,0], [-27,24], [5,25], [174,32], [150, 80]]).reshape(-1,1,2)

# SCENE MATCHING CONSTANTS
sceneMatchingAlgorithm = 'SIFT'
minMatchCount = 5