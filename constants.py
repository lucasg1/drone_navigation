import numpy as np

from enums import ImageInput

# MAIN CONSTANTS
isSingleTest = False

# CAPTURADOR CONSTANTS
inputType = ImageInput.VIDEO
videoFile = 'imagensTeste/teste4.mp4'

# ESTIMADOR CONSTANTS
templateImage = 'imagensTeste/experimento2Referencial25.jpg'
singleImageTest = 'golden.jpeg'
mapTemplatePoints = np.float32([[ 182 , 574 ],[ 73 , 440 ], [ 518 , 200 ], [ 871 , 528 ], [ 962 , 290 ]]).reshape(-1,1,2)
mapRealPoints = np.float32([[0,0], [-27,30], [80,80], [150,0], [177, 57]]).reshape(-1,1,2)

# SCENE MATCHING CONSTANTS
sceneMatchingAlgorithm = 'SIFT'
minMatchCount = 8