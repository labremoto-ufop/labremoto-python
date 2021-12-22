#!/usr/bin/env python
##############################################################################
#   Laboratorio Remoto de Robotica Movel - TCC
#   Arquivo de Controle
##############################################################################
#   Author: Paulo Felipe - paulof (at) ufop.edu.br
#
##############################################################################
#  Contem as funcoes de calculo de angulo e do controlador do robo
##############################################################################
import rospy
import os
import json
import numpy
import math
from math import atan2, acos, cos, sin, sqrt, pi
import cv2
from geometry_msgs.msg import Twist
from entities.ev3 import Ev3, Point
from nav_msgs.msg import Odometry
from datetime import datetime, timedelta
import sys, select, termios, tty


class RobotController:

    def __init__(self):
        self.odometry = Odometry()
        
    # Produto vetorial entre 2 vetores
    def crossProduct(self, vecA, vecB):
        return (vecA.x * vecB.y) - (vecA.y * vecB.x)

    # Produto escalar entre 2 vetores
    def dotProduct(self, vecA, vecB):
        return (vecA.x * vecB.x) + (vecA.y * vecB.y)

    # Pega o angulo alpha, que define a orientacao do robo atraves do produto escalar e
    # vetorial entre o objetivo e a posicao atual do robo
    #
    #   Matematicamente:
    #       alpha = atan2( A x B, A . B)
    #
    def getAlpha(self, ev3, goal):
        vectorEv3 = Point()
        vectorEv3.x = ev3.front.x - ev3.center.x
        vectorEv3.y = ev3.front.y - ev3.center.y
        vectorGoal = Point()
        vectorGoal.x = goal.x - ev3.center.x
        vectorGoal.y = goal.y - ev3.center.y
        return math.atan2(
            self.crossProduct(vectorEv3, vectorGoal),
            self.dotProduct(vectorEv3, vectorGoal),
        )

    # Inicializa o controlador PID, zerando as variaveis de soma e alpha
    def initPid(self):
        self.lastAlpha = 0
        self.alphaSum = 0
        self.lastAlphaAng = 0
        self.alphaSumAng = 0
        self.startTime = datetime.now()

    # Roda o PID
    def pidRun(self, graph, goal, pose, ev3, lastFlag, experimento):

        # Verifica se nao excedeu o tempo limite, caso tenha lanca uma excecao
        if datetime.now() > (self.startTime + timedelta(0, 50)):
            raise Exception("Tempo de execucao excedido")

        # Inicializa publisher para comandar a velocidade no ROS
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Variaveis do controlador
        # kP = 0.005
        # kPa = 0.256
        # kI = 0.001
        # kD = 0.002
        vMax = 0.8
        kP = experimento.parametros.kp
        kD = experimento.parametros.kd
        kI = experimento.parametros.ki
        kPa = experimento.parametros.kp * 45

        kPang = experimento.parametros.kp_ang
        kDang = experimento.parametros.kd_ang
        kIang = experimento.parametros.ki_ang

        # Seta as variaveis do experimento

        # Erro permitido pelo controlador
        err = 70

        # Calcula distancia e diferenca do angulo entre o robo e o objetivo
        rho = math.sqrt((goal.x - pose.x) ** 2 + (goal.y - pose.y) ** 2)
        alpha = self.getAlpha(ev3, goal)

        # Grava os erros
        experimento.linearError = rho
        experimento.angularError = alpha
        experimento.currentGoal = [goal.x,goal.y]

        # Verifica a condicao de parada
        if abs(rho) < err:
            return True, ()

        # Define velocidade linear
        self.alphaSum += rho
        linearVelocity = (
            kP * rho + kI * self.alphaSum + kDang * (rho - self.lastAlpha)
        )
        self.lastAlphaAng = alpha
        linearVelocity = -min(linearVelocity, vMax)

        # Calcula velocidade angular
        self.alphaSumAng += alpha
        angularVelocity = (
            kPang * alpha + kIang * self.alphaSumAng + kDang * (alpha - self.lastAlphaAng)
        )
        self.lastAlphaAng = alpha

        # Aplica velocidade no robo
        # print("PosRobo (%s,%s) | Objetivo (%s,%s) | Vel Linear = %s | Vel Angular = %s"
        #% (pose.x,pose.y,goal.x,goal.y,linearVelocity,angularVelocity))
        # print(linearVelocity, angularVelocity)
        twist = Twist()
        twist.linear.x = linearVelocity
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = angularVelocity
        pub.publish(twist)
        return False, (pose.x, pose.y, goal.x, goal.y, linearVelocity, angularVelocity)
    
    # Roda o PID
    def pidRunLinearizationFeedback(self, graph, goal, pose, ev3, lastFlag, experimento):

        # Verifica se nao excedeu o tempo limite, caso tenha lanca uma excecao
        if datetime.now() > (self.startTime + timedelta(0, 50)):
            raise Exception("Tempo de execucao excedido")

        # Inicializa publisher para comandar a velocidade no ROS
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Inicializa subscriber para ler a velocidade do ROS
        subscriber = rospy.Subscriber("odom", Odometry, self.odometryCallback)

        # Variaveis do controlador
        vMax = 0.8
        K = experimento.parametros.kp
        d = 1

        # Erro permitido pelo controlador
        err = 70


        vector = Point()
        u = Point()
        vector.x = ev3.front.x - ev3.center.x
        vector.y = ev3.front.y - ev3.center.y
        u.x = (ev3.center.x + 1) - ev3.center.x
        u.y = 0

        dot = (vector.x * u.x) + (vector.y * u.y)
        det = (vector.x * u.x) - (vector.y * u.y)
        
        #theta = atan2(det,dot)
        #theta = acos(((vector.x * u.x) + (vector.y * u.y)) / (sqrt(vector.x**2 + vector.y**2) * sqrt(u.x**2 + u.y**2)) )
        theta = atan2(vector.y, vector.x) - atan2(u.y, u.x)
        
        cosTheta = cos(theta)
        sinTheta = sin(theta)
        print("cosTheta: " + str(cosTheta))
        print("sinTheta: " + str(sinTheta))

        J = [
            [cosTheta, sinTheta],
            [-sinTheta/d, cosTheta/d]
        ]

        u = [K*(goal.x - pose.x), K*(goal.y - pose.y)]

        linearVelocity = cosTheta*u[0] + sinTheta*u[1]
        angularVelocity = (-sinTheta/d)*u[0] + (cosTheta/d)*u[1]

        #if(ev3.front.x > ev3.center.x):
        #    linearVelocity = linearVelocity*-1
        # Verifica a condicao de parada
        rho = math.sqrt((goal.x - pose.x) ** 2 + (goal.y - pose.y) ** 2)

        if abs(rho) < err:
            return True, ()

        # Grava os erros
        experimento.linearError = rho
        experimento.currentGoal = [goal.x,goal.y]

        # Aplica velocidade no robo
        # print("PosRobo (%s,%s) | Objetivo (%s,%s) | Vel Linear = %s | Vel Angular = %s"
        #% (pose.x,pose.y,goal.x,goal.y,linearVelocity,angularVelocity))
        # print(linearVelocity, angularVelocity)
        twist = Twist()
        twist.linear.x = -linearVelocity
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z =  angularVelocity
        pub.publish(twist)
        return False, (pose.x, pose.y, goal.x, goal.y, linearVelocity, angularVelocity)

    def stopRobot(self):
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

    def teleoperacao(self, instrucao):

        instrucoes = {2: (1, 0), 1: (-1, 0), 4: (0, 1), 3: (0, -1)}
        linearVel = 0.5
        angularVel = 0.5

        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        twist = Twist()
        twist.linear.x = instrucoes[instrucao][0] * linearVel
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = instrucoes[instrucao][1] * angularVel
        pub.publish(twist)

    # Roda as instrucoes
    def runInstrucao(self, graph, instrucao, pose, ev3, experimento):

        # Inicializa publisher para comandar a velocidade no ROS
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Inicializa subscriber para ler a velocidade do ROS
        subscriber = rospy.Subscriber("odom", Odometry, self.odometryCallback)

        twist = Twist()
        twist.linear.x = instrucao.velocidadeLinear
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = instrucao.velocidadeAngular
        pub.publish(twist)
        return False, (pose.x, pose.y, 0, 0, self.odometry.twist.twist.linear.x, self.odometry.twist.twist.angular.z)

    # Callback da velocidade
    def odometryCallback(self, data):
        self.odometry = data


    def matrixMulti(self, X, Y):
        result = [[0,0]]
        # iterate through rows of X
        for i in range(len(X)):
        # iterate through columns of Y
            for j in range(len(Y[0])):
                # iterate through rows of Y
                for k in range(len(Y)):
                    result[i][j] += X[i][k] * Y[k][j]
        return result