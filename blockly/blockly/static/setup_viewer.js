import { WebotsView } from 'https://cyberbotics.com/wwi/R2023b/WebotsView.js';

let webotsStreamURL = null;
globalThis.containerId = null;
let base_api_url = "https://robots.ulstu.ru:444/api/"
let reconnectAttempts = 0;
const MAX_RECONNECT_ATTEMPTS = 10;
let reconnectionTimer = null;
let isConnected = false;

const mobileDevice = /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
if (mobileDevice) {
  let head = document.getElementsByTagName('head')[0];

  let mobileCss = document.createElement('link');
  mobileCss.setAttribute('rel', 'stylesheet');
  mobileCss.setAttribute('type', 'text/css');
  mobileCss.setAttribute('href', 'https://www.cyberbotics.com/wwi/R2025a/css/wwi_mobile.css');
  head.appendChild(mobileCss);
}

function init() {
  fetchWebotsStreamURL();
}

function fetchWebotsStreamURL() {
  const urlParams = new URLSearchParams(window.location.search);
  const vsPort = window.location.port; // Extract vs_port from the current URL
  const worldPath = urlParams.get('world_path');

  if (!vsPort) {
    console.error("vsPort is null or undefined");
    return;
  }

  fetch(base_api_url + 'get-webots-stream-port/', {
    method: 'POST',
    headers: { 
      'Content-Type': 'application/json',
      'X-CSRFToken': getCookie('csrftoken')
    },
    credentials: 'include',
    body: JSON.stringify({ vs_port: vsPort, world_path: worldPath })
  })
  .then(response => response.json())
  .then(data => {
    if (data.webots_stream_port) {
      webotsStreamURL = `wss://robots.ulstu.ru:${data.webots_stream_port}`;
      console.log("Проверяем доступность WebSocket перед подключением...");
      setTimeout(checkWebSocketAvailability, 2000, webotsStreamURL);
    } else {
      console.error("Ошибка получения webots_stream_port:", data.detail);
      webotsStreamURL = 'ws://localhost:1235';
      setTimeout(checkWebSocketAvailability, 2000, webotsStreamURL);
    }
  })
  .catch(error => console.error("Ошибка при вызове API:", error));
}

function checkWebSocketAvailability(url, maxAttempts = 10) {
  let attempts = 0;
  
  function attemptConnection() {
    attempts++;
    console.log(`Попытка подключения ${attempts}/${maxAttempts}...`);
    
    const socket = new WebSocket(url);
    
    socket.onopen = function() {
      console.log("WebSocket доступен, показываем мир Webots");
      socket.close();
      show_world();
    };
    
    socket.onerror = function() {
      if (attempts < maxAttempts) {
        console.log("WebSocket недоступен, повторная попытка через 1 секунду...");
        setTimeout(attemptConnection, 1000);
      } else {
        console.log("Достигнуто максимальное количество попыток, пробуем показать мир в любом случае");
        show_world();
      }
    };
  }
  
  attemptConnection();
}

function getCookie(name) {
  let cookieValue = null;
  if (document.cookie && document.cookie !== '') {
    const cookies = document.cookie.split(';');
    for (let i = 0; i < cookies.length; i++) {
      const cookie = cookies[i].trim();
      if (cookie.substring(0, name.length + 1) === (name + '=')) {
        cookieValue = decodeURIComponent(cookie.substring(name.length + 1));
        break;
      }
    }
  }
  return cookieValue;
}

function show_world() {
  console.log("Start show world");

  let webotsView = document.getElementsByTagName("webots-view")[0];

  if (!webotsView) {
      console.error("Webots Viewer не найден! Убедитесь, что <webots-view> есть в HTML.");
      return;
  }

  console.log("Найден Webots Viewer:", webotsView);
  console.log("Методы Webots Viewer:", Object.keys(webotsView));

  if (typeof webotsView.connect !== "function") {
      console.error("Ошибка: `connect` не является функцией. Webots Viewer не загружен или версия устарела.");
      return;
  }

  const defaultThumbnail = "https://cyberbotics.com/wwi/R2023b/images/loading/default_thumbnail.png";
  const streamingMode = "x3d";

  webotsView.onready = onConnect;
  webotsView.ondisconnect = onDisconnect;
  
  // Override the default error handler to prevent the modal
  webotsView.onerror = function(event) {
    console.error("WebSocket error:", event);
    // Prevent default error handling
    event.preventDefault();
    event.stopPropagation();
    
    // If not already trying to reconnect, start reconnection process
    if (!reconnectionTimer) {
      handleDisconnection();
    }
    
    return false;
  };

  if (webotsStreamURL) {
      console.log("Подключаемся к Webots Stream:", webotsStreamURL);
      webotsView.connect(webotsStreamURL, 'x3d', false, mobileDevice, -1, defaultThumbnail);
  } else {
      console.log("Webots stream url is null");
  }
  
  // Set up a heartbeat to regularly check connection
  startConnectionHeartbeat();
}

function onConnect() {
  console.log("Webots View ready");
  isConnected = true;
  reconnectAttempts = 0; // Reset reconnect counter on successful connection
}

function onDisconnect() {
  console.log("Disconnected from Webots stream");
  isConnected = false;
  handleDisconnection();
}

function handleDisconnection() {
  // Clear any existing reconnection attempts
  if (reconnectionTimer) {
    clearTimeout(reconnectionTimer);
  }
  
  if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
    reconnectAttempts++;
    console.log(`Attempting to reconnect (${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS})...`);
    
    // Exponential backoff for reconnection attempts (1s, 2s, 4s, etc.)
    const backoffTime = Math.min(1000 * Math.pow(2, reconnectAttempts - 1), 30000); 
    
    reconnectionTimer = setTimeout(() => {
      reconnectionTimer = null;
      reconnectToWebots();
    }, backoffTime);
  } else {
    console.error("Maximum reconnection attempts reached. Please refresh the page.");
  }
}

function reconnectToWebots() {
  console.log("Attempting to reconnect to Webots stream...");
  
  // First check if the WebSocket is available
  checkWebSocketAvailability(webotsStreamURL, 3);
}

function startConnectionHeartbeat() {
  // Check connection status every 5 seconds
  setInterval(() => {
    const webotsView = document.getElementsByTagName("webots-view")[0];
    
    if (webotsView && typeof webotsView.isConnected === 'function') {
      const connectionStatus = webotsView.isConnected();
      
      if (!connectionStatus && isConnected) {
        console.log("Connection heartbeat detected disconnection");
        isConnected = false;
        handleDisconnection();
      } else if (connectionStatus && !isConnected) {
        isConnected = true;
        console.log("Connection restored");
      }
    }
  }, 5000);
}

function disconnect() {
  const webotsView = document.getElementsByTagName('webots-view')[0];
  if (webotsView) {
    webotsView.close();
  }
  
  if (reconnectionTimer) {
    clearTimeout(reconnectionTimer);
    reconnectionTimer = null;
  }
  
  isConnected = false;
}

init();
