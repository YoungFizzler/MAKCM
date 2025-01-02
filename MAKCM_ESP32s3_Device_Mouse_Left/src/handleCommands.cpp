#include "handleCommands.h"
#include "InitSettings.h"
#include "tasks.h"
#include <Arduino.h>
#include <USB.h>
#include <USBHIDMouse.h>
#include "USBSetup.h"
#include <esp_intr_alloc.h>
#include <cstring>
#include <atomic>
#include <mutex>
#include <RingBuf.h>
#include <map>
#include <vector>

// Atomic variables for mouse movement and button states
std::atomic<int> moveX(0);
std::atomic<int> moveY(0);
std::atomic<bool> isLeftButtonPressed(false);
std::atomic<bool> isRightButtonPressed(false);
std::atomic<bool> isMiddleButtonPressed(false);
std::atomic<bool> isForwardButtonPressed(false);
std::atomic<bool> isBackwardButtonPressed(false);
std::atomic<bool> serial0Locked(true);
std::atomic<bool> kmMoveCom(false);

// Task handles
extern TaskHandle_t mouseMoveTaskHandle;
extern TaskHandle_t ledFlashTaskHandle;

std::mutex commandMutex;

volatile bool deviceConnected = false;
bool usbReady = false;
bool processingUsbCommands = false;



// Create the ring buffers
void processRingBufferCommand(RingBuf<char, 620> &buffer);
RingBuf<char, 620> serial0RingBuffer;
RingBuf<char, 620> serial1RingBuffer;
int currentCommandIndex = 0;

int16_t mouseX = 0;
int16_t mouseY = 0;

const unsigned long ledFlashTime = 25; // Set The LED Flash timer in ms

const char *commandQueue[] = {
    "sendDeviceInfo",
    "sendDescriptorDevice",
    "sendEndpointDescriptors",
    "sendInterfaceDescriptors",
    "sendHidDescriptors",
    "sendIADescriptors",
    "sendEndpointData",
    "sendUnknownDescriptors",
    "sendDescriptorconfig"
};

// Command tables
CommandEntry serial0CommandTable[] = {
    {"DEBUG_", handleDebug},         // Handle debug commands and logging levels
    {"SERIAL_", handleSerial0Speed}  // Change serial port speed (115200-5000000)
};

CommandEntry debugCommandTable[] = {
    {"ESPLOG_", handleEspLog},                    // Log messages to Serial0
    {"PRINT_Parsed_Descriptors", printParsedDescriptors}, // Print USB descriptor info
    {"HID_Descriptors", [](const char* arg) { Serial1.print(arg); }} // Print HID descriptor data
};

// Recoil table structure
struct RecoilPattern {
    std::vector<int> pattern;  // Format: x, y, delay, x, y, delay, ...
    String name;
};

// Store recoil patterns
std::map<String, RecoilPattern> recoilPatterns;

CommandEntry normalCommandTable[] = {
    {"nozen.moveto", handleKmMoveto},        // Move mouse to absolute x,y coordinates
    {"nozen.getpos", handleKmGetpos},        // Get current mouse position
    {"nozen.left(1)", handleKmMouseButtonLeft1},    // Press left mouse button
    {"nozen.left(0)", handleKmMouseButtonLeft0},    // Release left mouse button
    {"nozen.right(1)", handleKmMouseButtonRight1},  // Press right mouse button
    {"nozen.right(0)", handleKmMouseButtonRight0},  // Release right mouse button
    {"nozen.middle(1)", handleKmMouseButtonMiddle1},// Press middle mouse button
    {"nozen.middle(0)", handleKmMouseButtonMiddle0},// Release middle mouse button
    {"nozen.side1(1)", handleKmMouseButtonForward1},// Press forward side button
    {"nozen.side1(0)", handleKmMouseButtonForward0},// Release forward side button
    {"nozen.side2(1)", handleKmMouseButtonBackward1},// Press back side button
    {"nozen.side2(0)", handleKmMouseButtonBackward0},// Release back side button
    {"nozen.wheel", handleKmWheel},          // Mouse wheel movement
    {"nozen.recoil.add", handleRecoilAdd},   // Add new recoil pattern: name{x,y,delay,...}
    {"nozen.recoil.delete", handleRecoilDelete}, // Delete a recoil pattern by name
    {"nozen.recoil.list", handleRecoilList}, // List all recoil patterns with full data
    {"nozen.recoil.get", handleRecoilGet},   // Get a specific recoil pattern by name
    {"nozen.recoil.names", handleRecoilNames},// List only the names of stored patterns
    {"nozen.print", handlePrint},            // Print a message to Serial0
    {"nozen.restart", handleRestart}         // Restart both ESPs
};

CommandEntry usbCommandTable[] = {
    {"USB_HELLO", handleUsbHello},           // Initialize USB connection
    {"USB_GOODBYE", handleUsbGoodbye},       // Clean disconnect of USB device
    {"USB_ISNULL", handleNoDevice},          // Handle no device connected state
    {"USB_sendDeviceInfo:", receiveDeviceInfo}, // Receive USB device information
    {"USB_sendDescriptorDevice:", receiveDescriptorDevice}, // Receive device descriptor
    {"USB_sendEndpointDescriptors:", receiveEndpointDescriptors}, // Receive endpoint info
    {"USB_sendInterfaceDescriptors:", receiveInterfaceDescriptors}, // Receive interface info
    {"USB_sendHidDescriptors:", receiveHidDescriptors}, // Receive HID descriptor data
    {"USB_sendIADescriptors:", receiveIADescriptors}, // Receive interface association data
    {"USB_sendEndpointData:", receiveEndpointData}, // Receive endpoint specific data
    {"USB_sendUnknownDescriptors:", receiveUnknownDescriptors}, // Handle unknown descriptors
    {"USB_sendDescriptorconfig:", receivedescriptorConfiguration} // Receive config descriptor
};

void processCommand(const char *command);

void trimCommand(char* command) {
    int len = strlen(command);
    while (len > 0 && (command[len - 1] == ' ' || command[len - 1] == '\n' || command[len - 1] == '\r')) {
        command[len - 1] = '\0';
        len--;
    }
}

void serial0RX() {
    while (Serial0.available() > 0) {
        char byte = Serial0.read();

        if (byte == '\r') {
            continue;
        }

        if (!serial0RingBuffer.isFull()) {
            serial0RingBuffer.push(byte);
        } else {
            Serial0.println("Serial0 ring buffer overflow detected.");
        }

        if (byte == '\n') {
            char commandBuffer[620];
            int commandIndex = 0;

            while (!serial0RingBuffer.isEmpty() && commandIndex < sizeof(commandBuffer) - 1) {
                serial0RingBuffer.pop(commandBuffer[commandIndex++]);
            }

            commandBuffer[commandIndex] = '\0';

            trimCommand(commandBuffer);

            if (strncmp(commandBuffer, "nozen.move", 10) == 0) {
                if (!kmMoveCom) {
                    kmMoveCom = true;
                    handleKmMoveCommand(commandBuffer);
                }
            } else {
                processCommand(commandBuffer);
            }
        }
    }
}

void serial1RX() {
    while (Serial1.available() > 0) {
        char byte = Serial1.read();

        if (byte == '\r') {
            continue;
        }

        if (!serial1RingBuffer.isFull()) {
            serial1RingBuffer.push(byte);
        } else {
            Serial0.println("Serial1 ring buffer overflow detected.");
        }

        if (byte == '\n') {
            char commandBuffer[620];
            int commandIndex = 0;

            while (!serial1RingBuffer.isEmpty() && commandIndex < sizeof(commandBuffer) - 1) {
                serial1RingBuffer.pop(commandBuffer[commandIndex++]);
            }

            commandBuffer[commandIndex] = '\0';

            trimCommand(commandBuffer);

            if (strncmp(commandBuffer, "nozen.move", 10) == 0 && !kmMoveCom) {
                handleKmMoveCommand(commandBuffer);
            } else {
                processCommand(commandBuffer);
            }
        }
    }
}


void processRingBufferCommand(RingBuf<char, 620> &buffer) {
    char commandBuffer[620];
    int commandIndex = 0;
    char byte;

    while (!buffer.isEmpty()) {
        buffer.pop(byte);
        
        Serial0.print("Popped Byte: '");
        Serial0.print(byte);
        Serial0.print("' (ASCII: ");
        Serial0.print((int)byte); 
       
        if (byte == '\n' || commandIndex >= 620 - 1) {
            break;
        }

        commandBuffer[commandIndex++] = byte;
    }

    commandBuffer[commandIndex] = '\0'; 
    trimCommand(commandBuffer);
    
    Serial0.print("Full Command: ");
    Serial0.println(commandBuffer);

    if (commandIndex > 0) {
        processCommand(commandBuffer);
    }
}


void handleKmMoveCommand(const char *command) {
    int x, y;

    sscanf(command + strlen("nozen.move") + 1, "%d,%d", &x, &y);

    {
        std::lock_guard<std::mutex> lock(commandMutex);
        moveX = x;
        moveY = y;
    }

    if (mouseMoveTaskHandle != NULL) {
        xTaskNotifyGive(mouseMoveTaskHandle);
    }

    kmMoveCom = false;
}

void ledFlashTask(void *parameter) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        digitalWrite(9, HIGH);
        vTaskDelay(ledFlashTime / portTICK_PERIOD_MS);
        digitalWrite(9, LOW);
        vTaskDelay(ledFlashTime / portTICK_PERIOD_MS);
    }
}

void notifyLedFlashTask() {
    xTaskNotifyGive(ledFlashTaskHandle);
}


void handleUsbHello(const char *command) {
    deviceConnected = true;
    usbReady = true;
    processingUsbCommands = true;
    currentCommandIndex = 0;
    sendNextCommand();
}

void handleUsbGoodbye(const char *command) {
    Serial0.println("USB Device disconnected. Restarting!");
    handleMove(0, 0);
    handleMouseButton(MOUSE_BUTTON_LEFT, false);
    handleMouseButton(MOUSE_BUTTON_RIGHT, false);
    handleMouseButton(MOUSE_BUTTON_MIDDLE, false);
    handleMouseButton(MOUSE_BUTTON_FORWARD, false);
    handleMouseButton(MOUSE_BUTTON_BACKWARD, false);
    handleMouseWheel(0);
    vTaskDelay(100);
    ESP.restart();
}

void sendNextCommand() {
    if (!processingUsbCommands || currentCommandIndex >= sizeof(commandQueue) / sizeof(commandQueue[0])) {
        return;
    }
    const char *command = commandQueue[currentCommandIndex];
    Serial1.println(command);
    currentCommandIndex++;
    if (currentCommandIndex >= sizeof(commandQueue) / sizeof(commandQueue[0])) {
        usbReady = false;
        processingUsbCommands = false;
        InitUSB();
        vTaskDelay(700);
        serial0Locked = false;
        Serial1.println("USB_INIT");
    }
}

void processCommand(const char *command) {
 
    for (const auto &entry : debugCommandTable) {
        if (strncmp(command, entry.command, strlen(entry.command)) == 0) {
            entry.handler(command);
            return;
        }
    }

    for (const auto &entry : serial0CommandTable) {
        if (strncmp(command, entry.command, strlen(entry.command)) == 0) {
            entry.handler(command);
            return;
        }
    }

    for (const auto &entry : usbCommandTable) {
        if (strncmp(command, entry.command, strlen(entry.command)) == 0) {
            entry.handler(command);
            return;
        }
    }

    if (!processingUsbCommands) {
        for (const auto &entry : normalCommandTable) {
            if (strncmp(command, entry.command, strlen(entry.command)) == 0) {
                entry.handler(command);
                return;
            }
        }
    }

    handleDebugcommand(command);
}


void handleEspLog(const char *command) {
    const char *message = command + strlen("ESPLOG_");
    if (strlen(message) > 0) {
        Serial0.println(message);
    } else {
        Serial0.println("ESPLOG_ command received, but no message to log.");
    }
}

void handleSerial0Speed(const char *command) {
    int speed;
    if (sscanf(command + strlen("SERIAL_"), "%d", &speed) == 1) {
        if (speed >= 115200 && speed <= 5000000) {
            Serial0.print("Setting Serial0 speed to: ");
            Serial0.println(speed);
            Serial0.end();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            Serial0.begin(speed);
            Serial0.onReceive(serial0ISR);
            Serial0.println("Serial0 speed change successful.");
        } else {
            Serial0.println("Speed was out of bounds. Min: 115200, Max: 5000000.");
        }
    } else {
        Serial0.println("Invalid SERIAL command. Expected format: SERIAL_<speed>");
    }
}

void handleDebug(const char *command) {
    int debugLevel;
    if (strcmp(command, "DEBUG_ON") == 0) {
        Serial1.println("DEBUG_ON");
    } else if (strcmp(command, "DEBUG_OFF") == 0) {
        Serial1.println("DEBUG_OFF");
    } else if (sscanf(command + strlen("DEBUG_"), "%d", &debugLevel) == 1) {
        Serial1.println(command);
    } else {
        Serial0.println("Invalid DEBUG command. Bytes received:");
        for (int i = 0; i < strlen(command); i++) {
            Serial0.print("Byte ");
            Serial0.print(i);
            Serial0.print(": '");
            Serial0.print(command[i]); 
            Serial0.print("' (ASCII: ");
            Serial0.print((int)command[i]);
            Serial0.println(")");
        }
    }
}


void handleDebugcommand(const char *command) {
    Serial0.println(command);
}

void handleNoDevice(const char *command)
{
    deviceConnected = false;
}

void mouseMoveTask(void *pvParameters) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int x, y;
        {
            std::lock_guard<std::mutex> lock(commandMutex);
            x = moveX;
            y = moveY;
        }
        handleMove(x, y);
    }
}

void handleKmMoveto(const char *command) {
    int x, y;
    sscanf(command + strlen("nozen.moveto") + 1, "%d,%d", &x, &y);
    handleMoveto(x, y);
}

void handleKmGetpos(const char *command) {
    handleGetPos();
}

void handleKmMouseButtonLeft1(const char *command) {
    if (!isLeftButtonPressed.exchange(true)) {
        handleMouseButton(MOUSE_BUTTON_LEFT, true);
    }
}

void handleKmMouseButtonLeft0(const char *command) {
    if (isLeftButtonPressed.exchange(false)) {
        handleMouseButton(MOUSE_BUTTON_LEFT, false);
    }
}

void handleKmMouseButtonRight1(const char *command) {
    if (!isRightButtonPressed.exchange(true)) {
        handleMouseButton(MOUSE_BUTTON_RIGHT, true);
    }
}

void handleKmMouseButtonRight0(const char *command) {
    if (isRightButtonPressed.exchange(false)) {
        handleMouseButton(MOUSE_BUTTON_RIGHT, false);
    }
}

void handleKmMouseButtonMiddle1(const char *command) {
    if (!isMiddleButtonPressed.exchange(true)) {
        handleMouseButton(MOUSE_BUTTON_MIDDLE, true);
    }
}

void handleKmMouseButtonMiddle0(const char *command) {
    if (isMiddleButtonPressed.exchange(false)) {
        handleMouseButton(MOUSE_BUTTON_MIDDLE, false);
    }
}

void handleKmMouseButtonForward1(const char *command) {
    if (!isForwardButtonPressed.exchange(true)) {
        handleMouseButton(MOUSE_BUTTON_FORWARD, true);
    }
}

void handleKmMouseButtonForward0(const char *command) {
    if (isForwardButtonPressed.exchange(false)) {
        handleMouseButton(MOUSE_BUTTON_FORWARD, false);
    }
}

void handleKmMouseButtonBackward1(const char *command) {
    if (!isBackwardButtonPressed.exchange(true)) {
        handleMouseButton(MOUSE_BUTTON_BACKWARD, true);
    }
}

void handleKmMouseButtonBackward0(const char *command) {
    if (isBackwardButtonPressed.exchange(false)) {
        handleMouseButton(MOUSE_BUTTON_BACKWARD, false);
    }
}

void handleKmWheel(const char *command) {
    int wheelMovement;
    sscanf(command + strlen("nozen.wheel") + 1, "%d", &wheelMovement);
    handleMouseWheel(wheelMovement);
}

void handleMove(int x, int y) {
    Mouse.move(x, y);
    mouseX += x;
    mouseY += y;
}

void handleMoveto(int x, int y) {
    Mouse.move(x - mouseX, y - mouseY);
    mouseX = x;
    mouseY = y;
}

void handleMouseButton(uint8_t button, bool press) {
    if (press)
        Mouse.press(button);
    else
        Mouse.release(button);
}

void handleMouseWheel(int wheelMovement) {
    Mouse.move(0, 0, wheelMovement);
}

void handleGetPos() {
    Serial0.println("km.pos(" + String(mouseX) + "," + String(mouseY) + ")");
}

void handleRecoilAdd(const char *command) {
    // Format: nozen.recoil.add(name){x,y,delay,x,y,delay,...}
    char name[32];
    char pattern[256];
    
    if (sscanf(command + strlen("nozen.recoil.add"), "(%[^)]){%[^}]}", name, pattern) == 2) {
        RecoilPattern newPattern;
        newPattern.name = String(name);
        
        // Parse the pattern string
        char *token = strtok(pattern, ",");
        while (token != NULL) {
            newPattern.pattern.push_back(atoi(token));
            token = strtok(NULL, ",");
        }
        
        // Validate pattern (must be multiple of 3: x,y,delay)
        if (newPattern.pattern.size() % 3 == 0) {
            recoilPatterns[newPattern.name] = newPattern;
            Serial0.println("Recoil pattern added: " + newPattern.name);
        } else {
            Serial0.println("Invalid pattern format. Must be x,y,delay triplets");
        }
    } else {
        Serial0.println("Invalid command format. Use: nozen.recoil.add(name){x,y,delay,x,y,delay,...}");
    }
}

void handleRecoilDelete(const char *command) {
    // Format: nozen.recoil.delete(name)
    char name[32];
    
    if (sscanf(command + strlen("nozen.recoil.delete"), "(%[^)])", name) == 1) {
        String patternName = String(name);
        if (recoilPatterns.erase(patternName) > 0) {
            Serial0.println("Recoil pattern deleted: " + patternName);
        } else {
            Serial0.println("Pattern not found: " + patternName);
        }
    } else {
        Serial0.println("Invalid command format. Use: nozen.recoil.delete(name)");
    }
}

void handleRecoilList(const char *command) {
    Serial0.println("Stored recoil patterns:");
    for (const auto &pattern : recoilPatterns) {
        Serial0.print(pattern.first + ": {");
        for (size_t i = 0; i < pattern.second.pattern.size(); i++) {
            Serial0.print(pattern.second.pattern[i]);
            if (i < pattern.second.pattern.size() - 1) {
                Serial0.print(",");
            }
        }
        Serial0.println("}");
    }
}

void handleRecoilGet(const char *command) {
    // Format: nozen.recoil.get(name)
    char name[32];
    
    if (sscanf(command + strlen("nozen.recoil.get"), "(%[^)])", name) == 1) {
        String patternName = String(name);
        auto it = recoilPatterns.find(patternName);
        
        if (it != recoilPatterns.end()) {
            Serial0.print(it->first + ": {");
            for (size_t i = 0; i < it->second.pattern.size(); i++) {
                Serial0.print(it->second.pattern[i]);
                if (i < it->second.pattern.size() - 1) {
                    Serial0.print(",");
                }
            }
            Serial0.println("}");
        } else {
            Serial0.println("Pattern not found: " + patternName);
        }
    } else {
        Serial0.println("Invalid command format. Use: nozen.recoil.get(name)");
    }
}

void handleRecoilNames(const char *command) {
    Serial0.println("Available recoil patterns:");
    for (const auto &pattern : recoilPatterns) {
        Serial0.println("- " + pattern.first);
    }
}

void handlePrint(const char *command) {
    // Format: nozen.print(message)
    char message[256];
    
    if (sscanf(command + strlen("nozen.print"), "(%[^)])", message) == 1) {
        Serial0.println(message);
    } else {
        Serial0.println("Invalid command format. Use: nozen.print(message)");
    }
}

void handleRestart(const char *command) {
    Serial0.println("Restarting both ESPs...");
    // Signal the other ESP to restart
    Serial1.println("RESTART");
    // Wait a bit for the message to be sent
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // Restart this ESP
    ESP.restart();
}
