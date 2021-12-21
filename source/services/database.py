#!/usr/bin/env python
import mysql.connector
from datetime import datetime, timedelta
import json
from entities.experimento import SessaoExperimento, SessaoExperimentoApontarParametros, ExperimentoData, SessaoExperimentoTrajetoriaParametros, SessaoExperimentoInstrucao
##############################################################################
#   Laboratorio Remoto de Robotica Movel - TCC
#   Arquivo de conexao com o banco de dados
##############################################################################
#   Author: Paulo Felipe - paulof (at) ufop.edu.br
#
##############################################################################
#
#
class Database():

    def initDatabase(self):
        self.conn = mysql.connector.connect(
            host="localhost",
            user="labremoto",
            password="Labremoto21##Pw",
            database="labremoto")
        self.conn.autocommit = True

    def getSessaoAtiva(self):
        queryString = "SELECT * FROM sessao WHERE ativo = 1"
        cursor = self.conn.cursor()
        cursor.execute(queryString)
        results = cursor.fetchall()
        return results

    def removeSessaoAtiva(self):
        queryString = "UPDATE sessao SET ativo = 0 WHERE ativo = 1"
        cursor = self.conn.cursor()
        cursor.execute(queryString)
        self.conn.commit()
        queryString = "UPDATE sessao_experimento SET ativo = 0 WHERE ativo = 1"
        cursor = self.conn.cursor()
        cursor.execute(queryString)
        self.conn.commit()

    def removeExperimentoAtivo(self):
        queryString = "UPDATE sessao_experimento SET ativo = 0 WHERE ativo = 1"
        cursor = self.conn.cursor()
        cursor.execute(queryString)
        self.conn.commit()


    def getExperimentosSessaoAtiva(self, codSessaoAtiva):
        queryString = "SELECT codigo, cod_sessao, cod_experimento, dt_inicio, ativo FROM sessao_experimento WHERE cod_sessao = %s"
        cursor = self.conn.cursor()
        cursor.execute(queryString, (codSessaoAtiva, ))
        result = cursor.fetchone()
        if(result == None):
            return None
        sessaoExperimento = SessaoExperimento()
        sessaoExperimento.codigo = result[0]
        sessaoExperimento.codSessao = result[1]
        sessaoExperimento.codExperimento = result[2]
        sessaoExperimento.dtInicio = result[3]
        sessaoExperimento.ativo = result[4]
        return sessaoExperimento

    def getExperimentoAtivo(self, codSessaoAtiva):
        queryString = "SELECT codigo, cod_sessao, cod_experimento, dt_inicio, ativo FROM sessao_experimento WHERE cod_sessao = %s AND ativo = true"
        cursor = self.conn.cursor()
        cursor.execute(queryString, (codSessaoAtiva, ))
        result = cursor.fetchone()
        if(result == None):
            return None
        sessaoExperimento = SessaoExperimento()
        sessaoExperimento.codigo = result[0]
        sessaoExperimento.codSessao = result[1]
        sessaoExperimento.codExperimento = result[2]
        sessaoExperimento.dtInicio = result[3]
        sessaoExperimento.ativo = result[4]
        return sessaoExperimento
    
    def getParametrosExperimentoApontar(self, codExperimento):
        queryString = "SELECT cod_sessao_experimento, algoritmo_busca, obstaculos, kp, kd, ki, "
        queryString += "tamanho_mapa_busca, tamanho_area_seguranca, dt_criacao, objetivo_x, objetivo_y, heuristica, kp_ang, kd_ang, ki_ang, tipo_controlador "
        queryString += "FROM experimento_apontar_parametros WHERE cod_sessao_experimento = %s"
        cursor = self.conn.cursor()
        cursor.execute(queryString, (codExperimento, ))
        result = cursor.fetchone()
        if(result == None):
            return None
        parametros = SessaoExperimentoApontarParametros()
        parametros.codSessaoExperimento = result[0]
        parametros.algoritmoBusca = result[1]
        parametros.obstaculos = result[2]
        parametros.kp = result[3]
        parametros.kd = result[4]
        parametros.ki = result[5]
        parametros.tamanhoMapaBusca = result[6]
        parametros.tamanhoAreaSeguranca = result[7]
        parametros.dtCriacao = result[8]
        parametros.objetivoX = result[9]
        parametros.objetivoY = result[10]
        parametros.heuristica = result[11]
        parametros.kp_ang = result[12]
        parametros.kd_ang = result[13]
        parametros.ki_ang = result[14]
        parametros.tipoControlador = result[15]
        return parametros

    def getParametrosExperimentoInstrucoes(self, codExperimento):
        queryString = "SELECT cod_sessao_experimento, obstaculos, kp, kd, ki, dt_criacao "
        queryString += "FROM experimento_trajetoria_parametros WHERE cod_sessao_experimento = %s"
        cursor = self.conn.cursor()
        cursor.execute(queryString, (codExperimento, ))
        result = cursor.fetchone()
        if(result == None):
            return None
        parametros = SessaoExperimentoTrajetoriaParametros()
        parametros.codSessaoExperimento = result[0]
        parametros.obstaculos = result[1]
        parametros.kp = result[2]
        parametros.kd = result[3]
        parametros.ki = result[4]
        parametros.dtCriacao = result[5]
        return parametros

    def getExperimentoInstrucoes(self, codExperimento):
        queryString = "SELECT codigo, cod_sessao_experimento, velocidade_linear, velocidade_angular, timer, dt_criacao, dt_inicializacao, dt_finalizacao "
        queryString += "FROM experimento_trajetoria_instrucoes WHERE cod_sessao_experimento = %s ORDER BY codigo ASC"
        cursor = self.conn.cursor(buffered=True)
        cursor.execute(queryString, (codExperimento, ))
        result = cursor.fetchall()
        if(result == None):
            return None
        instrucoes = []
        for instrucao in result:
            parametros = SessaoExperimentoInstrucao()
            parametros.codigo = instrucao[0]
            parametros.codSessaoExperimento = instrucao[1]
            parametros.velocidadeLinear = instrucao[2]
            parametros.velocidadeAngular = instrucao[3]
            parametros.timer = instrucao[4]
            parametros.dtCriacao = instrucao[5]
            parametros.dtInicializacao = instrucao[6]
            parametros.dtFinalizacao = instrucao[7]
            instrucoes.append(parametros)
        return instrucoes
    
    def getRodarExperimentoStatus(self):
        queryString = "SELECT valor FROM configuracoes WHERE id = 2"
        cursor = self.conn.cursor()
        cursor.execute(queryString)
        results = cursor.fetchone()
        return int(results[0])

    def setRodarExperimentoStatus(self, status):
        queryString = "UPDATE configuracoes SET valor = '" + str(status) + "' WHERE id = 2"
        cursor = self.conn.cursor(buffered=True)
        cursor.execute(queryString)
        self.conn.commit()

    
    def setExperimentoInativo(self):
        queryString = "UPDATE sessao_experimento SET ativo = 0 WHERE ativo = 1"
        cursor = self.conn.cursor()
        cursor.execute(queryString)
        self.conn.commit()

    def setExperimentoResults(self, experimentoAtivo, telemetry, data):
        queryString = "INSERT INTO experimento_resultados (cod_sessao_experimento, "
        queryString += "pos_x, pos_y, linear_vel, angular_vel, experimento_starttime, data, "
        queryString += "dt_criacao) VALUES (%s,%s,%s,%s,%s,%s,%s,%s)"

        queryData = (
        experimentoAtivo.codigo,
        telemetry[0],
        telemetry[1],
        telemetry[4],
        telemetry[5],
        data.starttime,
        json.dumps(data.__dict__),
        datetime.now())
        
        cursor = self.conn.cursor()
        cursor.execute(queryString, queryData)
        self.conn.commit()

    def setExperimentoStats(self, experimentoAtivo, experimentoStats):
        queryString = "UPDATE experimento_apontar_parametros SET estatisticas_busca = %s "
        queryString += "WHERE cod_sessao_experimento = %s"

        queryData = (
        json.dumps(experimentoStats.__dict__),
        experimentoAtivo.codigo)
        
        cursor = self.conn.cursor()
        cursor.execute(queryString, queryData)
        self.conn.commit()

    def getNextTeleoperacaoInstrucao(self):
        queryString = "SELECT codigo, cod_sessao, instrucao, dt_criacao FROM teleoperacao_instrucoes ORDER BY codigo ASC LIMIT 1"
        cursor = self.conn.cursor()
        cursor.execute(queryString)
        results = cursor.fetchone()
        return results
    
    def removeTeleoperacaoInstrucao(self, codigo):
        queryString = "DELETE FROM teleoperacao_instrucoes WHERE codigo = %s"
        queryData = (codigo, )
        cursor = self.conn.cursor()
        cursor.execute(queryString, queryData)
        self.conn.commit()
    
