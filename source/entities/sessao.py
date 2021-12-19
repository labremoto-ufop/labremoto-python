class Sessao():
        def __init__(self, sessaoArray = None):
            if sessaoArray is not None:
                self.id = sessaoArray[0]
                self.matricula = sessaoArray[1]
                self.ativo = sessaoArray[2]
                self.dtInicio = sessaoArray[3]
                self.dtFim = sessaoArray[4]
            else:
                self.id = None
                self.matricula = None
                self.ativo = None
                self.dtInicio = None
                self.dtFim = None