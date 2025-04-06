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

# Add a common prefix for API routes
@app.route('/api/save', methods=['POST'])
def save_program():
    print("Hit /api/save endpoint")  # Debug log
    try:
        data = request.json
        name = data.get('name', '').strip()
        xml = data.get('xml', '')
        if not name or not xml:
            return jsonify({"status": "error", "message": "Invalid data"})

        project_name = data.get('world_path', os.getenv('PROJECT_NAME', ''))
        PROGRAMS_DIR = f"/ulstu/repositories/{project_name}/controllers/blockly_controller/"
        file_path = os.path.join(PROGRAMS_DIR, f"{name}.xml")
        with open(file_path, "w") as f:
            f.write(xml)

        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/api/list', methods=['POST'])
def list_programs():
    print("Hit /api/list endpoint")  # Debug log
    try:
        data = request.json
        project_name = data.get('world_path', os.getenv('PROJECT_NAME', ''))
        PROGRAMS_DIR = f"/ulstu/repositories/{project_name}/controllers/blockly_controller/"
        programs = [f[:-4] for f in os.listdir(PROGRAMS_DIR) if f.endswith('.xml')]
        return jsonify({"programs": programs})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/api/load', methods=['POST'])
def load_program():
    print("Hit /api/load endpoint")  # Debug log
    try:
        data = request.json
        project_name = data.get('world_path', os.getenv('PROJECT_NAME', ''))
        PROGRAMS_DIR = f"/ulstu/repositories/{project_name}/controllers/blockly_controller/"
        name = data.get('name', '').strip()
        file_path = os.path.join(PROGRAMS_DIR, f"{name}.xml")
        if not os.path.exists(file_path):
            return jsonify({"status": "error", "message": "Program not found"})

        with open(file_path, "r") as f:
            xml = f.read()

        print(f"Loaded XML for program '{name}': {xml[:100]}...")  # Debug log (first 100 characters)
        return jsonify({"status": "ok", "xml": xml})
    except Exception as e:
        print(f"Error in /api/load: {e}")  # Debug log
        return jsonify({"status": "error", "message": str(e)})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=31415, debug=True)

