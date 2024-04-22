from flask import Flask, render_template
from flask import Flask, send_from_directory

app = Flask(__name__)


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/js/<path:path>')
def send_js(path):
    return send_from_directory('js', path)


@app.route('/assets/<path:path>')
def send_assets(path):
    return send_from_directory('../assets', path)

@app.route('/node_modules/<path:path>')
def send_node_modules(path):
    return send_from_directory('../node_modules', path)

if __name__ == '__main__':
    app.run(debug=True)

