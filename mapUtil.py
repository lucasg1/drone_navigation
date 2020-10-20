import map_constants

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