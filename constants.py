import numpy as np

from enums import ImageInput

# MAIN CONSTANTS
isSingleTest = False

# CAPTURADOR CONSTANTS
# inputType = ImageInput.VIDEO
inputType = ImageInput.DRONE_CAMERA
videoFile = 'imagensTeste/teste4.mp4'

# ESTIMADOR CONSTANTS
templateImage = 'imagensTeste/25.jpg'
singleImageTest = 'golden.jpeg'
mapTemplatePoints = np.float32([[261,184], [351, 595], [244, 702]]).reshape(-1,1,2)
mapRealPoints = np.float32([[0,0], [112, -30], [150, 0]]).reshape(-1,1,2)

# SCENE MATCHING CONSTANTS
sceneMatchingAlgorithm = 'SIFT'
minMatchCount = 5