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

        # Seta as variaveis do experimento

        # Erro permitido pelo controlador
        err = 100

        # Calcula distancia e diferenca do angulo entre o robo e o objetivo
        rho = math.sqrt((goal.x - pose.x) ** 2 + (goal.y - pose.y) ** 2)
        alpha = self.getAlpha(ev3, goal)

        # Verifica a condicao de parada
        if abs(rho) < err:
            return True, ()

        # Define velocidade linear
        # if(lastFlag == True):
        linearVelocity = -min(kP * rho, vMax)
        # else:
        #    linearVelocity = -vMax

        # Calcula velocidade angular
        self.alphaSum += alpha
        angularVelocity = (
            kPa * alpha + kI * self.alphaSum + kD * (alpha - self.lastAlpha)
        )
        self.lastAlpha = alpha

        # Grava os erros
        experimento.linearError = rho
        experimento.angularError = alpha

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
        twist.angular.z = -1 * angularVelocity
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

