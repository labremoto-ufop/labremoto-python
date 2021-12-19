#!/usr/bin/env python

##############################################################################
#   Laboratorio Remoto de Robotica Movel - TCC
#   Arquivo de gerenciamento de sessoes e experimentos
##############################################################################
#   Author: Paulo Felipe - paulof (at) ufop.edu.br
#
##############################################################################
#
#

from datetime import datetime, date
import time
from entities.sessao import Sessao
class SessaoService:

    sessaoAtiva = None

	# Construtor
    def __init__(self, db):
        self.db = db

    # Verifica se a sessao esta no tempo valido
    def checkSessaoTimeout(self):
        if self.sessaoAtiva is not None:
            now = datetime.now()
            if now > self.sessaoAtiva.dtFim:
                self.db.removeSessaoAtiva()
                return False
            return True
        return False

    # Pega a sessao ativa do laboratorio
    def getSessaoAtiva(self):
        #print("Carregando sessao ativa atual")
        sessoes = self.db.getSessaoAtiva()
        if len(sessoes) != 1:
            print("Nao foi encontrada sessao ativa no momento " + str(len(sessoes)) + " dormindo por alguns segundos.")
            time.sleep(5)
            return None
        self.sessaoAtiva = Sessao(sessoes[0])

        # Verifica timeout da sessao
        if(self.checkSessaoTimeout() == True):
         #   print("Sessao ativa carregada")
            return self.sessaoAtiva
        else:
            print("Sessao ativa terminada por timeout")
            return None

    # Pega os experimentos da sessao ativa
    def getExperimentosSessaoAtiva(self):
        if self.sessaoAtiva is None:
            return
        experimentos = self.db.getExperimentosSessaoAtiva(self.sessaoAtiva.id)
        self.experimentosSessaoAtiva = experimentos
        return self.experimentosSessaoAtiva
