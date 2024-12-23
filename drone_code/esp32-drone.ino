#include <WiFi.h>

// Access Point credentials
const char* ssid = "ESP32-AP";          // Name of the Wi-Fi network (SSID)
const char* password = "12345678";      // Password for the Wi-Fi network

// Static IP configuration for the Access Point
IPAddress local_IP(192, 168, 4, 1);     // Fixed IP for the ESP32
IPAddress gateway(192, 168, 4, 1);      // Gateway (same as the ESP32 IP for an AP)
IPAddress subnet(255, 255, 255, 0);     // Subnet mask

WiFiServer server(12345);               // Start a TCP server on port 12345

void setup() {
  Serial.begin(115200);

  // Configure the Access Point with a static IP
  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("Failed to configure static IP for AP");
    return;
  }

  // Start the Access Point
  WiFi.softAP(ssid, password);

  Serial.println("Access Point Started");
  Serial.print("ESP32 AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Start the server
  server.begin();
  Serial.println("Server started");
}

void loop() {
  // Check for incoming clients
  WiFiClient client = server.available();

  if (client) {
    Serial.println("New client connected");

    while (client.connected()) {

      // Send a response back to the client
      int data[15][3] = {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9},
        {10, 11, 12},
        {13, 14, 15},
        {16, 17, 18},
        {19, 20, 21},
        {22, 23, 24},
        {25, 26, 27},
        {28, 29, 30},
        {31, 32, 33},
        {34, 35, 36},
        {37, 38, 39},
        {40, 41, 42},
        {43, 44, 45}
      };
      
      client.write((uint8_t*)data, sizeof(data));
      Serial.println("Response sent");

      if (client.available()) {
        int data[3];
        client.read(reinterpret_cast<uint8_t*>(data), sizeof(data)); // Read the array
        Serial.printf("Received: [%d, %d, %d]\n", data[0], data[1], data[2]);
      }

    }

    client.stop();
    Serial.println("Client disconnected");
  }
}
