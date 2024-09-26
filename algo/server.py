import os
import sys
import time
sys.path.append(os.path.dirname(os.path.abspath(__file__ + "/..")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from flask import Flask, request, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

from algo.pathfinding.task1 import task1

task = task1()

@app.route('/generate_path', methods=['POST'])
def generate_path():
    message = request.get_json()
    path = task.generate_path(message)
    with open(f'path_{time.time()}.txt', 'w') as f:
        f.write(str(message))
        f.write(str(path))
    return jsonify(path)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)