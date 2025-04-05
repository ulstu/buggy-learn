from flask import Flask, request, jsonify, send_from_directory
import os

app = Flask(__name__)

@app.route('/')
def serve_index():
    return send_from_directory('/ulstu/server/static/build', 'index.html')

@app.route('/run', methods=['POST'])
def run_code():
    try:
        data = request.json
        code = data.get('code', '')
        project_name = data.get('world_path', os.getenv('PROJECT_NAME', ''))
        print(data, code)

        with open(f"/ulstu/repositories/{project_name}/controllers/blockly_controller/blockly_code.py", "w") as f:
            f.write(code)

        return jsonify({"status": "ok"})
    except:
        return jsonify({"status": "error"})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=31415)

