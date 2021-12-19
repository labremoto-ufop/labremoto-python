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

class WebdataService():
