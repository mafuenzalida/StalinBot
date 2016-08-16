#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import roslib
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from collections import Counter

import cv2
import cv
from cv_bridge import CvBridge, CvBridgeError

class VisionController:

	def __init__(self):
		self.__depth_img = rospy.Subscriber('/camera/depth/image', Image, self.__depth_handler)
		#self.__depth_img = rospy.Subscriber('/camera/depth/image_raw', Image, self.__depth_handler)
		self.__rgb_img = rospy.Subscriber('/camera/rgb/image_color', Image, self.__rgb_handler)
		self.bridge = CvBridge()
		self.current_cv_depth_image = np.zeros((1,1,3))
		self.current_cv_rgb_image = np.zeros((1,1,3))
		self.ready = False

	def __depth_handler(self, data):
		if self.ready == False:
			self.ready = True
		try:
			self.current_cv_depth_image = np.asarray(self.bridge.imgmsg_to_cv(data, "32FC1"))
			#self.current_cv_depth_image = np.asarray(self.bridge.imgmsg_to_cv2(data, "32FC1"))
			#rospy.loginfo("Imagen depth recibida " + str(self.current_cv_depth_image.shape))
			self.current_cv_depth_image = np.ma.masked_array(self.current_cv_depth_image, np.isnan(self.current_cv_depth_image))

		except CvBridgeError, e:
			print e

	def __rgb_handler(self, data):
		if self.ready == False:
			self.ready = True
		try:
			self.current_cv_rgb_image = np.asarray(self.bridge.imgmsg_to_cv(data, "bgr8"))
			#rospy.loginfo("Imagen RGB recibida " + str(self.current_cv_rgb_image.shape))

			# Concateno ambas imagenes solo para visualizacion
			I = self.current_cv_rgb_image
			D = np.zeros((I.shape[0], I.shape[1], 3), np.uint8)
			if self.current_cv_depth_image.shape[:2] == self.current_cv_rgb_image.shape[:2]:
				#rospy.loginfo("Imagen concatenada")
				D[:,:,0] = self.current_cv_depth_image * 20 #Solo para visualizacion
				D[:,:,1] = D[:,:,0]
				D[:,:,2] = D[:,:,1]

			I = I[40:400, 50:590]
			self.current_cv_rgb_image = self.current_cv_rgb_image[40:400, 50:590]
			cv2.imwrite("foto.jpg",I)
			#cv2.imshow("Image_RGB", np.concatenate((I,D), axis = 1))
			#cv2.imshow("Image_RGB", I)
			#cv2.waitKey(10)

		except CvBridgeError, e:
			print e

	def friend_dist_finder(self):
		n = 0
		data = [0] * 10
		data2 = [0] * 10
		while(True):
			depth_img = self.current_cv_depth_image
			frame = np.uint8(self.current_cv_rgb_image)
			#Operations on the frame
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			#hsv = self.current_cv_rgb_image
			yellow = cv2.inRange(hsv, np.array((0,100,100)), np.array((10,255,255)))
			blue = cv2.inRange(hsv, np.array((170,100,100)), np.array((180,255,255)))
			mask = cv2.add(yellow,blue)
			mask = cv2.erode(mask, None, iterations = 3)
			mask = cv2.dilate(mask, None, iterations = 15)
			contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
			cx = 220
			cy = 285
			#res = cv2.bitwise_and(frame, frame, mask = mask)
			depth = 0
			for cnt in range(0, len(contours)):
				[vx,vy,x,y] = cv2.fitLine(contours[cnt], cv2.cv.CV_DIST_L2, 0, 0.01, 0.01)
				
				if abs(vy/vx) > 3: ##Comprueba la orientacion
					M = cv2.moments(contours[cnt])
					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])
					x,y,w,h = cv2.boundingRect(contours[cnt])
					cx,cy = x+w/2, y+h/2

					##Depth objeto
					y_cont = 0
					x_cont = 0
					aux = np.mean(depth_img[y+y_cont:y+h-y_cont, x+x_cont:x+w-x_cont])
					if(aux == None):
						depth = 0
					else:
						depth = aux

					#print 'centroide en', (cx-640,cy-360)
					data[n] = (cx/10)*10
					data2[n] = (cy/10)*10
					n += 1
					if n == 10:
						n = 0
						cx = (Counter(data).most_common(1)[0][0]) + 4
						cy = (Counter(data2).most_common(1)[0][0]) + 4
						#rows, cols = res.shape[:2]
						print 'MARCA DETECTADA'
						return ((cx-320.0)/320.0), depth/1000.0
					#cv2.line(res, (cols-1,righty),(0,lefty),(0,0,255),1)
				width_camera = 320
				#desv = cx - width_camera
				#dist = float(int(100*abs(float(desv))/width_camera)) / 100

	def depth_finder(self):
		img = self.current_cv_depth_image
		#img es un array de 480x640 de floats que tienen la "distancia" segun el depth image 640x480

	def get_depth_center(self):
		depth_img = self.current_cv_depth_image
		##480*640
		line = depth_img[238:241,300:329]
		#return np.mean(line)/1000.0
		return np.mean(line)

	''' POR VER SI ES NECESARIO ALINEARSE
	def align_with_image(self):
		## If depth is > than a X depth then there isnt a wall, if there may be
	'''

	def get_center_sides_depth(self):
		depth_img = self.current_cv_depth_image
		line = depth_img[237:241,:]
		dl = np.mean(line[:,300:309])
		dr = np.mean(line[:,330:339])
		#return dr/1000.0, dl/1000.0
		return dr, dl
	def get_sides_depth(self, lr = 0):
		depth_img = self.current_cv_depth_image
		line = depth_img[237:241,:]
		dl = np.mean(line[:,96:104])
		#dl = np.mean(line[:,0:8])
		dr = np.mean(line[:,536:544])
		#dr = np.mean(line[:,631:639])
		if lr == 1:
			return dl/1000.0
		elif lr == 2:
			return dr/1000.0
		else:
			#return dr/1000.0, dl/1000.0
			return dr, dl

	def wait_image(self):
		print "Cargando Kinect"
		while self.ready == False:
			a = 3
		print "Listo"

	def tiene_cara(self, img):
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

	def p_negro(self, img):
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

	def p_blanco(self, img):
	    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	    ret,thresh1 = cv2.threshold(img,120,255,cv2.THRESH_BINARY)
	    ret,thresh2 = cv2.threshold(img,120,255,cv2.THRESH_BINARY_INV)
	    pb=100*(np.sum(thresh1))/(255*len(thresh1[0])*len(thresh1))
	    return pb

	def angle_cos(self, p0, p1, p2):
	    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
	    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

	def find_squares(self, img):
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
	                    max_cos = np.max([self.angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
	                    if max_cos < 0.1:
	                        squares.append(cnt)
	    return squares

	def cuadrado(self, img):
	    a=[0,0]
	    squares = self.find_squares(img)
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

	def p_amarillo(self, img):
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

	def p_rojo(self, img):
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

	def image_finder(self):
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
		img=cv2.imread('nodoblarizquierda.jpg')

		cara=tiene_cara(img)
		#print cara[0],'caras'
		if cara[0]>0:
		    #print cara
		    img = img[cara[2]:cara[2]+cara[4],cara[1]:cara[1]+cara[3]]    
		    pb = self.p_blanco(img)
		    pn = self.p_negro(img)
		    #print pb,'%blanco'
		    #print pn,'%negro'
		    if pn>2:
		        print 'CLON 2'
		        return 2
		    else:
		        if pb>14:
		            print 'CLON 1'
		            return 1
		        else:
		        	print 'AVERAGEMAN'
		            return 0
		else:
		    cuadrados = self.cuadrado(img) # [NUMERO DE CUADRADOS, FOTO DEL CUADRADO]
		    if cuadrados[0]>0:
		        pr = self.p_rojo(cuadrados[1])
		        pa = self.p_amarillo(cuadrados[1])
		        if pr>3 or pa>3:
		            #print pr,'rojo y',pa,'amarillo'
		            if pr>3*pa:
		                if pr>80:
		                    print 'PUERTA ROJA'
		                    return 3
		                else:
		                    print 'LLAVE ROJA'
		                    return 4
		            elif pa>3*pr:
		                if pa>80:
		                    print 'PUERTA AMARILLA'
		                    return 5
		                else:
		                    print 'LLAVE AMARILLA'
		                    return 6
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
		                        print 'DOBLAR DERECHA'
		                        return 7
		                    else:
		                        print 'DOBLAR IZQUIERDA'   
		                        return 8                 
		                else:                
		                    if (cx/float(len(thresh2[0]))) > 0.5:
		                        print 'NO DOBLAR DERECHA'
		                        return 9
		                    else:
		                        print 'NO DOBLAR IZQUIERDA'
		                        return 10
		    else:
		        print '0 cuadrados y 0 caras', 