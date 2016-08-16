import cv2
import numpy as np

def tiene_cara(img):
    a=[0,0,0,0,0]
    faceCascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.6,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )
    a[0] = len(faces)
    for (x, y, w, h) in faces:
        a[1] = x
        a[2] = y
        a[3] = w
        a[4] = h
    return a

def p_negro(img):
    cimg = cv2.medianBlur(img,5)
    hsv = cv2.cvtColor(cimg, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv,np.array((0,0,0)),np.array((179,50,100)))
    blue = cv2.inRange(hsv,np.array((0,0,0)),np.array((179,50,100)))
    mask = cv2.add(yellow,blue)
    mask = cv2.erode(mask,None,iterations = 3)
    mask = cv2.dilate(mask,None,iterations = 15)
    res = cv2.bitwise_and(img,img,mask= mask)
    pn=(100*np.sum(mask))/(255*len(mask[0])*len(mask))
    return pn

def p_blanco(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret,thresh1 = cv2.threshold(img,120,255,cv2.THRESH_BINARY)
    ret,thresh2 = cv2.threshold(img,120,255,cv2.THRESH_BINARY_INV)
    pb=100*(np.sum(thresh1))/(255*len(thresh1[0])*len(thresh1))
    return pb

def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

def find_squares(img):
    img = cv2.GaussianBlur(img, (5, 5), 0)
    squares = []
    for gray in cv2.split(img):
        for thrs in xrange(0, 255, 26):
            if thrs == 0:
                bin = cv2.Canny(gray, 0, 50, apertureSize=5)
                bin = cv2.dilate(bin, None)
            else:
                retval, bin = cv2.threshold(gray, thrs, 255, cv2.THRESH_BINARY)
            contours, hierarchy = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                cnt_len = cv2.arcLength(cnt, True)
                cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
                if len(cnt) == 4 and cv2.contourArea(cnt) > 1000 and cv2.isContourConvex(cnt):
                    cnt = cnt.reshape(-1, 2)
                    max_cos = np.max([angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
                    if max_cos < 0.1:
                        squares.append(cnt)
    return squares

def cuadrado(img):
    a=[0,0]
    squares = find_squares(img)
    a[0]=len(squares)
    #print squares[0]
    if len(squares)>1:
        #print squares
        menory=min(squares[0][0][1],squares[0][2][1])
        mayory=max(squares[0][0][1],squares[0][2][1])
        menorx=min(squares[0][0][0],squares[0][2][0])
        mayorx=max(squares[0][0][0],squares[0][2][0])
        #print menory, mayory, menorx, mayorx
        ada=0
        while menory<10 or mayory>(len(img)-10) or menorx<10 or mayorx>(len(img[0])-10):
            ada=ada+1
            if ada==len(squares):
                return [0,img]
            menory=min(squares[ada][0][1],squares[ada][2][1])
            mayory=max(squares[ada][0][1],squares[ada][2][1])
            menorx=min(squares[ada][0][0],squares[ada][2][0])
            mayorx=max(squares[ada][0][0],squares[ada][2][0])            
        #print [menory, mayory,min(squares[0][0][0],squares[0][2][0]),max(squares[0][0][0],squares[0][2][0])]
        #img=img[min(squares[0][0][1],squares[0][2][1]):max(squares[0][0][1],squares[0][2][1]),min(squares[0][0][0],squares[0][2][0]):max(squares[0][0][0],squares[0][2][0])]
        #img= img[squares[0][0][1]:squares[0][2][1],squares[0][0][0]:squares[0][2][0]]
        img=img[menory:mayory,menorx:mayorx]
        #cv2.imshow('squares', img)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()        
        a[1] = img
    return a

def p_amarillo(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv,np.array((20,100,100)),np.array((30,255,255)));
    blue = cv2.inRange(hsv,np.array((20,100,100)),np.array((30,255,255)));
    mask = cv2.add(yellow,blue)
    mask = cv2.erode(mask,None,iterations = 3)
    mask = cv2. dilate(mask,None,iterations = 10)
    #PORCENTAJE DE AMARILLO
    pa=100*(np.sum(mask))/(255*len(mask[0])*len(mask))
    #BINARIZA POR AMARILLO
    res = cv2.bitwise_and(img,img, mask= mask)
    #cv2.imshow('squares', res)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return pa

def p_rojo(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv,np.array((0,100,100)),np.array((10,255,255)))
    blue = cv2.inRange(hsv,np.array((170,100,100)),np.array((180,255,255)))
    mask = cv2.add(yellow,blue)
    mask = cv2.erode(mask,None,iterations = 3)
    mask = cv2. dilate(mask,None,iterations = 10)
    #PORCENTAJE DE ROJO
    pr=100*(np.sum(mask))/(255*len(mask[0])*len(mask))
    #BINARIZA POR ROJO
    res = cv2.bitwise_and(img,img, mask= mask)
    #cv2.imshow('squares', res)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return pr

#img=cv2.imread('averageman.jpg')
#img=cv2.imread('averageman_cloned01.jpg')
#img=cv2.imread('averageman_cloned02.jpg')
#img=cv2.imread('AVERAGE2.png')
#img=cv2.imread('puerta_roja.jpg')
#img=cv2.imread('llave_roja.jpg')
#img=cv2.imread('puerta_amarilla.jpg')
#img=cv2.imread('llave_amarilla.jpg')
#img=cv2.imread('doblarderecha.jpg')
#img=cv2.imread('nodoblarderecha.jpg')
#img=cv2.imread('doblarizquierda.jpg')
img=cv2.imread('IMGS/nodoblarizquierda.jpg')


cara=tiene_cara(img)
#print cara[0],'caras'
if cara[0]>0:
    #print cara
    img = img[cara[2]:cara[2]+cara[4],cara[1]:cara[1]+cara[3]]    
    pb = p_blanco(img)
    pn = p_negro(img)
    #print pb,'%blanco'
    #print pn,'%negro'
    if pn>2:
        print ('CLON 2')
    else:
        if pb>14:
            print ('CLON 1')
        else:
            print ('AVERAGEMAN')
else:
    cuadrados = cuadrado(img) # [NUMERO DE CUADRADOS, FOTO DEL CUADRADO]
    if cuadrados[0]>0:
        pr = p_rojo(cuadrados[1])
        pa = p_amarillo(cuadrados[1])
        if pr>3 or pa>3:
            #print pr,'rojo y',pa,'amarillo'
            if pr>3*pa:
                if pr>80:
                    print ('PUERTA ROJA')
                else:
                    print ('LLAVE ROJA')
            elif pa>3*pr:
                if pa>80:
                    print ('PUERTA AMARILLA')
                else:
                    print ('LLAVE AMARILLA')
        else:
            cuadrados[1] = cv2.cvtColor(cuadrados[1], cv2.COLOR_BGR2GRAY)
            ret,thresh2 = cv2.threshold(cuadrados[1],130,255,cv2.THRESH_BINARY_INV)            
            thresh2 = cv2.erode(thresh2,None,iterations = 2)
            thresh2 = cv2.dilate(thresh2,None,iterations = 15)           
            pn=100*np.sum(thresh2)/(255*len(thresh2[0])*len(thresh2))            
            #print pn,'porcentaje negro'          
            if pn>40:
                ret,thresh2 = cv2.threshold(cuadrados[1],90,255,cv2.THRESH_BINARY_INV)          
                thresh2 = cv2.erode(thresh2,None,iterations = 3)
                thresh2 = cv2.dilate(thresh2,None,iterations = 30)
                pn=100*np.sum(thresh2)/(255*len(thresh2[0])*len(thresh2))            
                #print pn,'porcentaje negro'                  
                M = cv2.moments(thresh2)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if pn<18:
                    if (cx/float(len(thresh2[0]))) > 0.5:
                        print ('DOBLAR DERECHA')
                    else:
                        print ('DOBLAR IZQUIERDA')                    
                else:                
                    if (cx/float(len(thresh2[0]))) > 0.5:
                        print ('NO DOBLAR DERECHA')
                    else:
                        print ('NO DOBLAR IZQUIERDA')
    else:
        print ('0 cuadrados y 0 caras') 


