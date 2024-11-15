// Import required libraries
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <esp_camera.h>
#include "fpv_car.h"

Servo myservo;

// Replace with your network credentials
const char* ssid = "etienne";
const char* password = "et140898";

// Direction
const int servo_dir = 2;

// Moteur
const int motor_pwm = 12;
const int motor_fwd = 13;
const int motor_bwd = 15;

String sliderValue = "0";
String throttleValue = "0";

int  acceleration;

const char* PARAM_INPUT = "value";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
// Port 81 pour le flux MJPEG
WiFiServer mjpegServer(81);

    // img {display: block; margin: 10px auto; max-width: 100%; height: auto;}
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP Web Server</title>
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 2.3rem;}
    p {font-size: 1.9rem;}
    body {max-width: 400px; margin:0px auto; padding-bottom: 25px;}
    .slider { -webkit-appearance: none; margin: 14px; width: 360px; height: 25px; background: #FFD65C;
      outline: none; -webkit-transition: .2s; transition: opacity .2s;}
    .slider::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; width: 35px; height: 35px; background: #003249; cursor: pointer;}
    .slider::-moz-range-thumb { width: 35px; height: 35px; background: #003249; cursor: pointer; } 
  </style>
</head>
<body>
  <h2>Go vroom vroom</h2>
  <img src="http://%ESP32_IP%:81" alt="Flux vidéo MJPEG" />
  <h3>Direction</h3>
  <p><span id="textSliderValue">%SLIDERVALUE%</span></p>
  <p><input type="range" onchange="updateSliderPWM(this)" id="pwmSlider" min="0" max="180" value="%SLIDERVALUE%" step="1" class="slider"></p>
  <h3>Throttle</h3>
  <p><span id="textThrottleValue">%THROTTLEVALUE%</span></p>
  <p><input type="range" onchange="updateThrottlePWM(this)" id="pwmThrottle" min="-255" max="255" value="%THROTTLEVALUE%" step="1" class="slider"></p>
<script>
function updateSliderPWM(element) {
  var sliderValue = document.getElementById("pwmSlider").value;
  document.getElementById("textSliderValue").innerHTML = sliderValue;
  console.log(sliderValue);
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/slider?value="+sliderValue, true);
  xhr.send();
}
function updateThrottlePWM(element) {
  var throttleValue = document.getElementById("pwmThrottle").value;
  document.getElementById("textThrottleValue").innerHTML = throttleValue;
  console.log(throttleValue);
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/throttle?value="+throttleValue, true);
  xhr.send();
}
</script>
</body>
</html>
)rawliteral";

// Replaces placeholder with button section in your web page
String processor(const String& var){
  //Serial.println(var);
  if (var == "ESP32_IP") {
    return WiFi.localIP().toString();
  }
  if (var == "SLIDERVALUE"){
    return sliderValue;
  }
  if (var == "THORTTLEVALUE"){
    return throttleValue;
  }
  return String();
}

void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Initialisation de la caméra avec résolution
  config.frame_size = FRAMESIZE_SVGA; // SVGA : 800x600
  // config.frame_size = FRAMESIZE_QVGA; // QVGA : 320x240
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Initialisation de la caméra
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error while camera init: 0x%x", err);
    ESP.restart();
  }
}

// Gestionnaire de flux MJPEG
void handleJpegStream(AsyncWebServerRequest* request) {
  Serial.println("Starting MJPEG stream...");
  AsyncWebServerResponse* response = request->beginChunkedResponse("multipart/x-mixed-replace; boundary=frame",
    [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Capture failed !");
        return 0;
      }
      Serial.println("Capture succeeded !");
      size_t len = fb->len;
      if (len > maxLen) len = maxLen;
      memcpy(buffer, fb->buf, len);
      esp_camera_fb_return(fb);
      Serial.printf("Frame size: %d bytes\n", len);
      return len;
    });

  if (response == nullptr) {
    Serial.println("Failed to create chunked response!");
  } else {
    Serial.println("Response created successfully.");
  }

  response->addHeader("Access-Control-Allow-Origin", "*");
  request->send(response);

  Serial.println("Response sent.");
  delay(100);
}

void handleSingleCapture(AsyncWebServerRequest* request) {
  Serial.println("Capturing single frame...");
  camera_fb_t *fb = esp_camera_fb_get();
  
  if (!fb) {
    Serial.println("Single capture failed !");
    request->send(500, "text/plain", "Capture failed");
    return;
  }

  Serial.println("Single capture succeeded !");
  request->send_P(200, "image/jpeg", fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleSimpleStream(AsyncWebServerRequest* request) {
  Serial.println("Starting simple MJPEG stream...");
  
  AsyncWebServerResponse* response = request->beginChunkedResponse(
    "multipart/x-mixed-replace; boundary=frame",
    [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      // Génère une simple chaîne "Hello world" pour tester
      const char* testFrame = "--frame\r\nContent-Type: text/plain\r\n\r\nHello world!\r\n";
      size_t len = strlen(testFrame);
      if (len > maxLen) len = maxLen;
      memcpy(buffer, testFrame, len);
      return len;
    }
  );

  if (response == nullptr) {
    Serial.println("Failed to create chunked response!");
  } else {
    Serial.println("Simple response created successfully.");
  }

  request->send(response);
}

void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);

//  Servo for direction
	if (!myservo.attached()) {
		myservo.setPeriodHertz(50); // standard 50 hz servo
		myservo.attach(servo_dir, 500, 2500);
	}
	myservo.write(90);

// H bridge for motor
  pinMode(motor_pwm, OUTPUT);
  pinMode(motor_fwd, OUTPUT);
  pinMode(motor_bwd, OUTPUT);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP Local IP Address
  Serial.println(WiFi.localIP());

  startCamera();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/slider?value=<inputMessage>
  server.on("/slider", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/slider?value=<inputMessage>
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      sliderValue = inputMessage;
      myservo.write(sliderValue.toInt());
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.print("Direction : ");
    Serial.println(inputMessage);
    request->send(200, "text/plain", "OK");
  });

  // Send a GET request to <ESP_IP>/throttle?value=<inputMessage>
  server.on("/throttle", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/throttle?value=<inputMessage>
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      acceleration = inputMessage.toInt();
      if (acceleration > 0) {
        digitalWrite(motor_fwd, HIGH);
        digitalWrite(motor_bwd, LOW);
      }
      else {
        digitalWrite(motor_fwd, LOW);
        digitalWrite(motor_bwd, HIGH);
      }
      analogWrite(motor_pwm, abs(acceleration));
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.print("throttle : ");
    Serial.println(inputMessage);
    request->send(200, "text/plain", "OK");
  });

  // server.on("/stream", HTTP_GET, handleJpegStream);
  // server.on("/stream", HTTP_GET, handleSimpleStream);
  // server.on("/capture", HTTP_GET, handleSingleCapture);
  
  // Start server
  server.begin();
  Serial.println("asynch server started on port 80");

  mjpegServer.begin();
  Serial.println("MJPEG server started on port 81");
}

void loop() {
  WiFiClient client = mjpegServer.available(); // Vérifie si un client est connecté
  if (client) {
      Serial.println("Client connected for MJPEG stream.");

      // En-tête HTTP pour le flux MJPEG
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
      client.println();

      while (client.connected()) {
          // Capture une image de la caméra
          camera_fb_t *fb = esp_camera_fb_get();
          if (fb) {
              // En-têtes pour une image JPEG
              client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
              client.write(fb->buf, fb->len); // Envoie les données JPEG
              client.println();
              esp_camera_fb_return(fb); // Libère le framebuffer
              // delay(100); // Ajoute un délai pour limiter les FPS
          } else {
              Serial.println("Failed to capture frame.");
          }

          // Vérifie si le client est encore connecté
          if (!client.connected()) {
              Serial.println("Client disconnected.");
              break;
          }
      }

      // Ferme la connexion lorsque le client se déconnecte
      client.stop();
      Serial.println("MJPEG client stopped.");
  }
}