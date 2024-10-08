#include <WiFi.h>
#include <WiFiUdp.h>

// Replace with your network credentials
const char* ssid = "Sense Semi";         // WiFi SSID (network name)
const char* password = "SSIT@504";       // WiFi password

// Define the UDP port and receiver IP address for communication
const int udpPort = 1234;                // UDP port for communication
const char* receiverIP = "192.168.0.126"; // IP address of the main unit (receiver)

// Define a unique sender ID to identify each sender device
const int senderID = 1;                  // Sender ID (1 for Sender 1, change to 2 for Sender 2)

// Initialize WiFi and UDP object
WiFiUDP udp;                             // Create a WiFiUDP instance for UDP communication

// Define relay pin connections for different appliances
const int CL = 2;                        // Pin connected to Ceiling Light relay
const int TL = 3;                        // Pin connected to Tube Light relay
const int BL = 4;                        // Pin connected to Bulb Light relay
const int FAN = 5;                       // Pin connected to Fan relay

void setup() {
  Serial.begin(9600);                    // Initialize Serial communication at 9600 baud rate
  
  // Set the relay pins as output for controlling the appliances
  pinMode(CL, OUTPUT);
  pinMode(TL, OUTPUT);
  pinMode(BL, OUTPUT);
  pinMode(FAN, OUTPUT);

  // Initially, turn all appliances off
  digitalWrite(CL, LOW);
  digitalWrite(TL, LOW);
  digitalWrite(BL, LOW);
  digitalWrite(FAN, LOW);

  // Connect to the WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { // Wait for WiFi connection
    delay(500);                           // Wait for half a second
    Serial.print(".");                    // Print a dot for connection progress
  }
  Serial.println("Connected to WiFi");    // Notify successful WiFi connection

  // Start the UDP service
  udp.begin(udpPort);
}

void loop() {
  // Send the current status of the relays to the receiver periodically
  sendStatusToReceiver();

  // Check if any UDP packets (commands) are received from the main unit
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];             // Buffer to store the incoming packet
    int len = udp.read(incomingPacket, 255); // Read the incoming packet into buffer
    if (len > 0) incomingPacket[len] = 0; // Null-terminate the packet string

    // Process the received command to control the relays
    processCommand(String(incomingPacket));
  }

  delay(5000);                            // Delay 5 seconds before sending the next status (adjustable)
}

void sendStatusToReceiver() {
  // Create an array to store the current statuses (on/off) of the relays
  int statuses[4] = {
    digitalRead(CL),                      // Read the status of the Ceiling Light relay
    digitalRead(TL),                      // Read the status of the Tube Light relay
    digitalRead(BL),                      // Read the status of the Bulb Light relay
    digitalRead(FAN)                      // Read the status of the Fan relay
  };

  // Create a packet that contains the sender ID and relay statuses
  uint8_t packet[sizeof(senderID) + sizeof(statuses)];
  memcpy(packet, &senderID, sizeof(senderID)); // Copy sender ID to the packet
  memcpy(packet + sizeof(senderID), statuses, sizeof(statuses)); // Copy statuses to the packet

  // Send the packet to the main unit (receiver) using UDP
  udp.beginPacket(receiverIP, udpPort);   // Begin the UDP packet
  udp.write(packet, sizeof(packet));      // Write the packet data
  udp.endPacket();                        // End the UDP packet transmission

  Serial.println("Status sent to receiver"); // Print status confirmation
}

void processCommand(String command) {
  // Check which command was received and act accordingly (turn relays on or off)
  if (command == "sender1cloff") {
    digitalWrite(CL, LOW);                // Turn Ceiling Light OFF
    Serial.println("Ceiling Light OFF");  // Log the action to Serial
  } else if (command == "sender1clon") {
    digitalWrite(CL, HIGH);               // Turn Ceiling Light ON
    Serial.println("Ceiling Light ON");   // Log the action to Serial
  } else if (command == "sender1tloff") {
    digitalWrite(TL, LOW);                // Turn Tube Light OFF
    Serial.println("Tube Light OFF");     // Log the action to Serial
  } else if (command == "sender1tlon") {
    digitalWrite(TL, HIGH);               // Turn Tube Light ON
    Serial.println("Tube Light ON");      // Log the action to Serial
  } else if (command == "sender1bloff") {
    digitalWrite(BL, LOW);                // Turn Bulb Light OFF
    Serial.println("Bulb Light OFF");     // Log the action to Serial
  } else if (command == "sender1blon") {
    digitalWrite(BL, HIGH);               // Turn Bulb Light ON
    Serial.println("Bulb Light ON");      // Log the action to Serial
  } else if (command == "sender1fanoff") {
    digitalWrite(FAN, LOW);               // Turn Fan OFF
    Serial.println("Fan OFF");            // Log the action to Serial
  } else if (command == "sender1fanon") {
    digitalWrite(FAN, HIGH);              // Turn Fan ON
    Serial.println("Fan ON");             // Log the action to Serial
  }

  // Log the received command to Serial
  Serial.println("Command received and processed: " + command);
}
