##############################################################################
#   Laboratorio Remoto de Robotica Movel - TCC
#   Arquivo de Processamento de Imagens
##############################################################################
#   Author: Paulo Felipe - paulof (at) ufop.edu.br
#
##############################################################################
#  Contem as funcoes relacionadas ao processamento basico de imagens do programa
##############################################################################
from collections import deque
from imutils.video import VideoStream
from math import atan2, cos, sin, sqrt, pi
from pathPlanningUtils import astar, bestFirstSearch, breadthFirstSearch, jumpPoint
import argparse
import numpy as np
import argparse
import cv2
import imutils
import time

class ImageProcessingUtils():
	# Retorna o frame da camera
	def getFrame(self, videoSource):
		frame = videoSource.read()
		frame = frame[1]
		return frame

	# Transforma o frame de RGB para HSV
	def frameToHsv(self, frame):
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		#cv2.imshow("blurred", blurred)
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		return hsv

	# Aplica a mascara pre-definida para mostrar os objetos
	def applyMaskToFrame(self, hsv, lowerBound, upperBound):
		mask = cv2.inRange(hsv, lowerBound, upperBound)
		#cv2.imshow("mask1", mask)
		mask = cv2.erode(mask, None, iterations=2)
		#cv2.imshow("mask2", mask)
		mask = cv2.dilate(mask, None, iterations=2)
		#cv2.imshow("mask3", mask)
		return mask

	# Retorna os contornos encontrados na imagem pos aplicacao da mascara
	def getObjectsContours(self, mask):
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
		return imutils.grab_contours(cnts)
	
	# Detecta a cor de determinado pixel na tela da camera
	def cvEventHandler(self, event, x, y, flags, params):
		if event == cv2.EVENT_RBUTTONDOWN:
			hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
			color = hsv[y,x]
			print("Cor HSV nas coord. [ %s, %s ]" % (x,y))
			print(color)
		if event == cv2.EVENT_LBUTTONDOWN:
			self.getImageMap(self.graph,x, y, 16, 3, None)

	# Mostra a Imagem na Tela
	def printImage(self, frame, graph):
		cv2.namedWindow("Robotica")
		cv2.setMouseCallback("Robotica", self.cvEventHandler)
		#cv2.imshow("Trajetoria", graph)
		self.frame = frame
		self.graph = graph
		#cv2.imshow("Robotica", frame)
		return cv2.waitKey(1) & 0xFF

	# Adiciona o espaco de seguranca no mapa
	# Esse espaco e pre-definido para evitar que o robo colida com os obstaculos
	def checkRobotSafeRange(self, map, mapImg, x, y, safeScale):
		for i in range(x-1):
			for j in range(y-1):
				for k in range(1,safeScale):
					if(map[i][j] != 0):
						continue
					# Checa pos a direita
					if(k+i < x):
						if(mapImg[j][i+k][1] > 60 and mapImg[j][i+k][0] != 255 and mapImg[j][i+k][2] < 60):
							map[i,j] = 10
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa pos a esquerda
					if(k-1 > 0):
						if(mapImg[j][i-k][1] > 60 and mapImg[j][i-k][0] != 255 and mapImg[j][i-k][2] < 60):
							map[i,j] = 10
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa pos em cima
					if(j-k > 0):
						if(mapImg[j-k][i][1] > 60 and mapImg[j-k][i][0] != 255 and mapImg[j-k][i][2] < 60):
							map[i,j] = 10
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa pos embaixo
					if(j+k < y):
						if(mapImg[j+k][i][1] > 60 and mapImg[j+k][i][0] != 255 and mapImg[j+k][i][2] < 60):
							map[i,j] = 10
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa na diagonal em cima direita
					if(k+i < x and j-k > 0):
						if(mapImg[j-k][i+k][1] > 60 and mapImg[j-k][i+k][0] != 255 and mapImg[j-k][i+k][2] < 60):
							map[i,j] = 10
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa na diagonal em cima esquerda
					if(k-1 > 0 and j-k > 0):
						if(mapImg[j-k][i-k][1] > 60 and mapImg[j-k][i-k][0] != 255 and mapImg[j-k][i-k][2] < 60):
							map[i,j] = 10
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa na diagonal embaixo direita
					if(k+i < x and j+k < y):
						if(mapImg[j+k][i+k][1] > 60 and mapImg[j+k][i+k][0] != 255 and mapImg[j+k][i+k][2] < 60):
							map[i,j] = 10
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa na diagonal embaixo equerda
					if(k-i < x and j+k < y):
						if(mapImg[j+k][i-k][1] > 60 and mapImg[j+k][i-k][0] != 255 and mapImg[j+k][i-k][2] < 60):
							map[i,j] = 10
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					
		return map, mapImg

	# Gera o mapa utilizado para as instrucoes
	def getInstrucoesMap(self, img, scale, safeScale, experimento, experimentoStats):
		x = 1280/scale
		y = 720/scale
		map = np.zeros((x,y))
		mapImg = img.copy()
		mapImg = cv2.resize(mapImg,(x, y))
		for i in range(x-1):
			for j in range(y-1):
				if(mapImg[j][i][1] > 60 and mapImg[j][i][0] != 255):
					map[i,j] = safeScale+1
		map, mapImg = self.checkRobotSafeRange(map, mapImg, x, y, safeScale)
		experimentoStats.map = map.tolist()
		experimentoStats.path = []
		experimentoStats.pathLen = 0
		self.mapScale = scale
		mapImg = cv2.resize(mapImg,(1280, 720))
		return mapImg

	# Gera o mapa utilizado pelo pathplanning
	def getImageMap(self, img, goalX, goalY, scale, safeScale, experimento, experimentoStats):
		x = 1280/scale
		y = 720/scale
		goalX = int(goalX/scale)
		goalY = int(goalY/scale)
		map = np.zeros((x,y))
		mapImg = img.copy()
		mapImg = cv2.resize(mapImg,(x, y))
		for i in range(x-1):
			for j in range(y-1):
				if(mapImg[j][i][1] > 60 and mapImg[j][i][0] != 255):
					map[i,j] = safeScale+1
		map, mapImg = self.checkRobotSafeRange(map, mapImg, x, y, safeScale)
		experimentoStats.map = map.tolist()
		if(map[goalX, goalY] != 0):
			print("Caminho obstruido")
		else:
			#print(experimento.parametros.__dict__)
			experimentoStats.initTimestamp = round(time.time() * 1000)
			maxIterations = 200
			if(experimento.parametros.algoritmoBusca == 1):
				path = astar(map, (int(self.ev3.front.x/scale),int(self.ev3.front.y/scale)), (goalX,goalY), maxIterations, experimento, experimentoStats)
			elif(experimento.parametros.algoritmoBusca == 2):
				path = breadthFirstSearch(map, (int(self.ev3.front.x/scale),int(self.ev3.front.y/scale)), (goalX,goalY), maxIterations, experimento, experimentoStats)
			elif(experimento.parametros.algoritmoBusca == 3):
				path = bestFirstSearch(map, (int(self.ev3.front.x/scale),int(self.ev3.front.y/scale)), (goalX,goalY), maxIterations, experimento, experimentoStats)
			else:
				path = jumpPoint(map, (int(self.ev3.front.x/scale),int(self.ev3.front.y/scale)), (goalX,goalY), maxIterations, experimento, experimentoStats)
			experimentoStats.finalTimestamp = round(time.time() * 1000)
			for coord in path:
				cv2.line(mapImg,coord,coord,(255,0,0),1)
			
			experimentoStats.path = path
			experimentoStats.pathLen = len(path)
			self.trajetoria = True
			self.path = path
			self.goal = (goalX, goalY)
			self.mapScale = scale
			self.updateTrajetoria = True
			mapImg = cv2.resize(mapImg,(1280, 720))

			#cv2.imshow("Mapeamento", mapImg)
		return mapImg

	# Reconhecimento dos pontos do objeto
	def recognizeObject(self, contours, frame, objLabel):
		if len(contours) > 0:
			c = max(contours, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			if radius > 2:
				return (x,y)
			return (-1, -1)
		return (-1, -1)

	# Imprime o contorno dos objetos
	def printWalls(self, frame, graph, contours):
		for c in contours:
			if(cv2.contourArea(c) < 1e3):
				continue
			ratio = 1
			# compute the center of the contour, then detect the name of the
			# shape using only the contour
			M = cv2.moments(c)
			cX = int((M["m10"] / M["m00"]) * ratio)
			cY = int((M["m01"] / M["m00"]) * ratio)

			# multiply the contour (x, y)-coordinates by the resize ratio,
			# then draw the contours and the name of the shape on the image
			c = c.astype("float")
			c *= ratio
			c = c.astype("int")
			#cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
			cv2.drawContours(graph, [c], -1, (59, 255, 59), 2, cv2.FILLED)
			cv2.fillPoly(graph, pts =[c], color=(59,255,59))
			#self.getOrientation(c, frame)


	# Imprime a seta de direcao no grafico principal
	def printRobot(self, ev3, frame, graph, scale):
		cv2.circle(graph, (int(ev3.center.x), int(ev3.center.y)), 4*scale, (30, 30, 30), -1)
		cv2.arrowedLine(graph, (int(ev3.center.x), int(ev3.center.y)), (int(ev3.front.x), int(ev3.front.y)), (255,0,0), 3, cv2.LINE_AA)
		cv2.arrowedLine(frame, (int(ev3.center.x), int(ev3.center.y)), (int(ev3.front.x), int(ev3.front.y)), (255,0,0), 3, cv2.LINE_AA)
			
	# Imprime as linhas do grafico na tela
	def printGraph(self, graph):
		height, width, channels = graph.shape
		for x in range(0, width -1, width/50):
			cv2.line(graph, (x, 0), (x, height), (100, 100, 100), 1, 1)
		for y in range(0, height -1, height/50):
			cv2.line(graph, (0, y), (width, y), (100, 100, 100), 1, 1)

	def init(self):
		self.trajetoria = False
		self.updateTrajetoria = False