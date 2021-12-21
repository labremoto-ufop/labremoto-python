from datetime import datetime

class SessaoExperimento():
        def __init__(self):
                self.codigo = 0
                self.codSessao = 0
                self.codExperimento = 0
                self.dtInicio = datetime.now()
                self.ativo = False
                self.parametros = None
                self.instrucoes = None
                
class SessaoExperimentoApontarParametros():
        def __init__(self):
                self.codSessaoExperimento = 0
                self.algoritmoBusca = 0
                self.tipoControlador = 1
                self.obstaculos = True
                self.kp = 1
                self.kd = 1
                self.ki = 1
                self.kp_ang = 1
                self.kd_ang = 1
                self.ki_ang = 1
                self.objetivoX = 0
                self.objetivoY = 0
                self.tamanhoMapaBusca = 0
                self.tamanhoAreaSeguranca = 0
                self.heuristica = 0
                self.dtCriacao = datetime.now()


class SessaoExperimentoTrajetoriaParametros():
        def __init__(self):
                self.codSessaoExperimento = 0
                self.obstaculos = True
                self.kp = 1
                self.kd = 1
                self.ki = 1
                self.dtCriacao = datetime.now()

class SessaoExperimentoInstrucao():
        def __init__(self):
                self.codigo = 0                
                self.codSessaoExperimento = 0
                self.tipo = 1
                self.velocidadeLinear = 0
                self.velocidadeAngular = 0
                self.timer = 0
                self.dtInicializacao = 0
                self.dtFinalizacao = 0
                self.dtCriacao = datetime.now()


class ExperimentoData():
        def __init__(self):
                self.starttime = 0