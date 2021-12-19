##############################################################################
#   Laboratorio Remoto de Robotica Movel - TCC
#   Arquivo do Servidor WEB
##############################################################################
#   Author: Paulo Felipe - paulof (at) ufop.edu.br
#
##############################################################################
#  Contem o servidor flask para trocar informacoes com o laboratorio remoto
##############################################################################
from flask import Flask, render_template, Response
from flask_cors import CORS
import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__)
CORS(app)
@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
	app.run(debug=True, threaded=True)
	
   	 