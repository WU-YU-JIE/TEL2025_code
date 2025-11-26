from flask import Flask, render_template_string, request, jsonify
import os
import time

app = Flask(__name__)

MSG_FILE = "/home/tel/msg.txt"
VERSION_FILE = "/home/tel/version.txt"

# 初始化版本控制
if not os.path.exists(VERSION_FILE):
    with open(VERSION_FILE, "w") as f:
        f.write("0")


def load_version():
    with open(VERSION_FILE, "r") as f:
        return int(f.read().strip())


def save_version(v):
    with open(VERSION_FILE, "w") as f:
        f.write(str(v))


def load_content():
    if os.path.exists(MSG_FILE):
        with open(MSG_FILE, "r", encoding="utf-8") as f:
            return f.read()
    return ""


def save_content(content):
    with open(MSG_FILE, "w", encoding="utf-8") as f:
        f.write(content)


def now_text():
    return time.strftime("%Y-%m-%d %H:%M:%S")


# ========== HTML 模板（你提供的原版） ==========
TEMPLATE = r"""
<html lang="zh-TW"> 
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>文字傳輸平台</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Microsoft JhengHei', Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        
        .container {
            max-width: 800px;
            margin: 0 auto;
            background: white;
            border-radius: 15px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
            padding: 30px;
        }
        
        h1 {
            color: #333;
            text-align: center;
            margin-bottom: 10px;
        }
        
        .status-bar {
            background: #f0f0f0;
            padding: 10px;
            border-radius: 8px;
            margin-bottom: 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        
        .status-indicator {
            display: flex;
            align-items: center;
            gap: 8px;
        }
        
        .status-dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background: #4CAF50;
            animation: pulse 2s infinite;
        }
        
        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.5; }
            100% { opacity: 1; }
        }
        
        .ip-info {
            font-size: 14px;
            color: #666;
        }
        
        textarea {
            width: 100%;
            min-height: 400px;
            padding: 15px;
            border: 2px solid #ddd;
            border-radius: 8px;
            font-size: 16px;
            resize: vertical;
            transition: border-color 0.3s;
        }
        
        textarea:focus {
            outline: none;
            border-color: #667eea;
        }
        
        .copy-button {
            width: 100%;
            padding: 15px;
            margin-top: 20px;
            font-size: 18px;
            font-weight: bold;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            transition: all 0.3s;
            background: linear-gradient(135deg, #48bb78 0%, #38a169 100%);
            color: white;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 10px;
        }
        
        .copy-button:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 20px rgba(72, 187, 120, 0.4);
        }
        
        .copy-button.copied {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            animation: success 0.5s;
        }
        
        @keyframes success {
            0% { transform: scale(1); }
            50% { transform: scale(1.05); }
            100% { transform: scale(1); }
        }
        
        .sync-indicator {
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 8px 16px;
            background: rgba(0, 0, 0, 0.7);
            color: white;
            border-radius: 20px;
            font-size: 12px;
            opacity: 0;
            transition: opacity 0.3s;
        }
        
        .sync-indicator.show {
            opacity: 1;
        }
        
        .last-update {
            text-align: center;
            color: #888;
            font-size: 14px;
            margin-top: 15px;
        }
        
        .char-count {
            text-align: right;
            color: #888;
            font-size: 14px;
            margin-top: 5px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>📝 文字傳輸平台</h1>
        
        <div class="status-bar">
            <div class="status-indicator">
                <div class="status-dot"></div>
                <span id="connectionStatus">連線中</span>
            </div>
            <div class="ip-info">
                伺服器: {{ server_ip }}:{{ port }}
            </div>
        </div>
        
        <textarea id="textContent" placeholder="在此輸入或貼上文字...">{{ content }}</textarea>
        
        <div class="char-count">
            字數: <span id="charCount">0</span>
        </div>
        
        <button class="copy-button" onclick="copyText()" id="copyBtn">
            <span>📋</span>
            <span id="copyBtnText">複製全部文字</span>
        </button>
        
        <div class="last-update" id="lastUpdate">
            {% if last_update %}
            最後更新: {{ last_update }}
            {% endif %}
        </div>
    </div>
    
    <div class="sync-indicator" id="syncIndicator">同步中...</div>
    
    <script>
        let currentVersion = {{ version }};
        let isTyping = false;
        let typingTimer;
        const textArea = document.getElementById('textContent');
        const charCount = document.getElementById('charCount');
        const syncIndicator = document.getElementById('syncIndicator');

        updateCharCount();

        textArea.addEventListener('input', function() {
            isTyping = true;
            clearTimeout(typingTimer);
            updateCharCount();
            saveText();

            typingTimer = setTimeout(() => {
                isTyping = false;
            }, 500);
        });

        function saveText() {
            const content = textArea.value;
            
            fetch('/save', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ content: content })
            })
            .then(response => response.json())
            .then(data => {
                currentVersion = data.version;
                showSyncIndicator();
                updateLastUpdate(data.last_update);
            });
        }

        function copyText() {
            const copyBtn = document.getElementById('copyBtn');
            const copyBtnText = document.getElementById('copyBtnText');
            
            navigator.clipboard.writeText(textArea.value)
                .then(() => {
                    copyBtn.classList.add('copied');
                    copyBtnText.textContent = '已複製！';
                    setTimeout(() => {
                        copyBtn.classList.remove('copied');
                        copyBtnText.textContent = '複製全部文字';
                    }, 2000);
                });
        }

        function updateCharCount() {
            charCount.textContent = textArea.value.length;
        }

        function showSyncIndicator() {
            syncIndicator.classList.add('show');
            setTimeout(() => {
                syncIndicator.classList.remove('show');
            }, 1000);
        }

        function updateLastUpdate(time) {
            const lastUpdate = document.getElementById('lastUpdate');
            if (time) {
                lastUpdate.textContent = '最後更新: ' + time;
            }
        }

        setInterval(() => {
            if (isTyping) return;

            fetch('/check-update')
                .then(response => response.json())
                .then(data => {
                    if (data.version !== currentVersion) {
                        currentVersion = data.version;
                        textArea.value = data.content;
                        updateCharCount();
                        updateLastUpdate(data.last_update);
                    }
                });
        }, 1000);
    </script>
</body>
</html>
"""


# ========== 路由 ==========
@app.route("/")
def index():
    server_ip = request.host.split(":")[0]
    port = request.host.split(":")[1] if ":" in request.host else "8080"
    return render_template_string(
        TEMPLATE,
        server_ip=server_ip,
        port=port,
        content=load_content(),
        version=load_version(),
        last_update=now_text(),
    )


@app.route("/save", methods=["POST"])
def save():
    data = request.get_json()
    content = data.get("content", "")
    save_content(content)

    v = load_version() + 1
    save_version(v)

    return jsonify({
        "status": "ok",
        "version": v,
        "last_update": now_text()
    })


@app.route("/check-update")
def check_update():
    return jsonify({
        "version": load_version(),
        "content": load_content(),
        "last_update": now_text()
    })


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
