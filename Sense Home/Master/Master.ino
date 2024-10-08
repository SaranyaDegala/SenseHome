#include <WiFi.h>
#include <WiFiUdp.h>

// Replace with your network credentials
const char* ssid = "Sense Semi";        // WiFi SSID (network name)
const char* password = "SSIT@504";      // WiFi password

// Define the UDP port for communication
const int udpPort = 1234;               // UDP port for sending/receiving messages
WiFiUDP udp;                            // Create a WiFiUDP instance for UDP communication

// Define the IP addresses of the sender devices
const char* sender1IP = "192.168.0.108"; // IP address of Sender 1
const char* sender2IP = "192.168.0.135"; // IP address of Sender 2

// Variables for heartbeat tracking
unsigned long lastHeartbeatTimeSender1 = 0; // Last time a heartbeat was received from Sender 1
unsigned long lastHeartbeatTimeSender2 = 0; // Last time a heartbeat was received from Sender 2
const unsigned long heartbeatTimeout = 10000; // Timeout period for heartbeat (10 seconds)

void setup() {
  Serial.begin(9600);                     // Initialize Serial communication at 9600 baud rate

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);             // Begin WiFi connection
  while (WiFi.status() != WL_CONNECTED) { // Wait until connected to WiFi
    delay(500);                           // Wait for half a second
    Serial.print(".");                    // Print progress indicator
  }
  Serial.println("Connected to WiFi");    // Notify successful WiFi connection

  // Start UDP service
  udp.begin(udpPort);                     // Start listening on the defined UDP port
}

void loop() {
  // Receive status updates from sender units
  int packetSize = udp.parsePacket();     // Check if a UDP packet is received
  if (packetSize) {                       // If a packet is available
    uint8_t buffer[255];                  // Buffer to hold the incoming packet data
    int len = udp.read(buffer, 255);      // Read the packet into buffer
    if (len > 0) {
      buffer[len] = '\0';                 // Null-terminate the received string
      processStatusUpdate(buffer, len);   // Process the received data
    }
  }

  // Check if a heartbeat timeout has occurred
  checkHeartbeatTimeout();                // Check if heartbeats have timed out

  // Example: Sending a command via Serial input to control Sender 1 or Sender 2
  if (Serial.available()) {               // If there's data from Serial (user input)
    String command = Serial.readStringUntil('\n'); // Read the command from Serial input
    if (command.startsWith("sender1")) {  // If command is for Sender 1
      sendCommand(command, sender1IP);    // Send the command to Sender 1
    } else if (command.startsWith("sender2")) { // If command is for Sender 2
      sendCommand(command, sender2IP);    // Send the command to Sender 2
    }
  }
}

// Function to process status updates or heartbeat messages
void processStatusUpdate(uint8_t* buffer, int len) {
  String receivedMessage = String((char*)buffer); // Convert received buffer to string

  // Check for heartbeat messages
  if (receivedMessage.startsWith("sender1heartbeat")) {
    updateHeartbeat(1);                   // Update heartbeat for Sender 1
  } else if (receivedMessage.startsWith("sender2heartbeat")) {
    updateHeartbeat(2);                   // Update heartbeat for Sender 2
  } else {
    // Process status update packet (non-heartbeat messages)
    int senderID;                         // Variable to hold sender ID
    int statuses[4];                      // Array to hold relay statuses (Ceiling Light, Tube Light, Bulb, Fan)

    // Extract sender ID and status from received packet
    memcpy(&senderID, buffer, sizeof(senderID));           // Copy the sender ID from the buffer
    memcpy(statuses, buffer + sizeof(senderID), sizeof(statuses)); // Copy the status values from the buffer

    // Print the status information
    Serial.print("Status from Sender ");
    Serial.print(senderID);
    Serial.print(": CL=");                // Ceiling Light status
    Serial.print(statuses[0] ? "ON" : "OFF");
    Serial.print(", TL=");                // Tube Light status
    Serial.print(statuses[1] ? "ON" : "OFF");
    Serial.print(", BL=");                // Bulb status
    Serial.print(statuses[2] ? "ON" : "OFF");
    Serial.print(", FAN=");               // Fan status
    Serial.println(statuses[3] ? "ON" : "OFF");
  }
}

// Function to update the last heartbeat time for each sender
void updateHeartbeat(int senderID) {
  if (senderID == 1) {
    lastHeartbeatTimeSender1 = millis();  // Update heartbeat time for Sender 1
    Serial.println("Heartbeat received from Sender 1");  // Log the heartbeat
  } else if (senderID == 2) {
    lastHeartbeatTimeSender2 = millis();  // Update heartbeat time for Sender 2
    Serial.println("Heartbeat received from Sender 2");  // Log the heartbeat
  }
}

// Function to check if heartbeat timeouts have occurred
void checkHeartbeatTimeout() {
  unsigned long currentTime = millis();   // Get the current time

  // Check for Sender 1 heartbeat timeout
  if (currentTime - lastHeartbeatTimeSender1 > heartbeatTimeout) {
    Serial.println("Warning: No heartbeat received from Sender 1 within the timeout period");
    // Handle sender 1 offline situation (e.g., turn off its relays, send alert, etc.)
  }

  // Check for Sender 2 heartbeat timeout
  if (currentTime - lastHeartbeatTimeSender2 > heartbeatTimeout) {
    Serial.println("Warning: No heartbeat received from Sender 2 within the timeout period");
    // Handle sender 2 offline situation
  }
}

// Function to send a command to a specific sender
void sendCommand(String command, const char* senderIP) {
  uint8_t commandPacket[255];             // Create a buffer for the command packet
  command.toCharArray((char*)commandPacket, 255); // Convert the command string to a char array

  // Send the command to the specified sender IP over UDP
  udp.beginPacket(senderIP, udpPort);     // Begin sending a UDP packet to the sender IP
  udp.write(commandPacket, strlen((char*)commandPacket)); // Write the command to the packet
  udp.endPacket();                        // End the UDP packet transmission

  Serial.println("Command sent: " + command); // Log the sent command
}
