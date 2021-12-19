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
import os
import json
import numpy
import math
from datetime import datetime, timedelta
from utils.pathPlanningUtils import Queue
import sys, select, termios, tty
import time



class InstrucoesService:
    def init(self):
        self.openInstrucoes = Queue()
        self.closedInstrucoes = Queue()
        self.currentInstrucao = None
        print("=============")
        print("Servico de instrucoes inicializado")

    def loadInstrucoes(self, instrucoes):
        print("Carregando instrucoes")
        for instrucao in instrucoes:
            self.openInstrucoes.put(instrucao)
        print(str(self.openInstrucoes.size()) + " instrucoes carregadas")

    def getInstrucao(self):
        if self.currentInstrucao is not None:
            if (
                self.currentInstrucao.dtInicializacao + self.currentInstrucao.timer
            ) < round(time.time() * 1000):
                self.closedInstrucoes.dtFinalizacao = round(time.time() * 1000)
                self.closedInstrucoes.put(self.currentInstrucao)
                self.currentInstrucao = None
            else:
                print(self.currentInstrucao.dtInicializacao + self.currentInstrucao.timer)
                print(time.time() * 1000)
                return self.currentInstrucao

        if not self.openInstrucoes.empty():
            self.currentInstrucao = self.openInstrucoes.get()
            self.currentInstrucao.dtInicializacao = round(time.time() * 1000)
            return self.currentInstrucao
        else:
            return None

