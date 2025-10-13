"""
Simple test script for IR Sensor Monitor Arduino
Tests communication protocol and sensor monitoring functionality

Usage:
1. Upload 02_IR_sensor_monitor.ino to Arduino
2. Connect Arduino to PC via USB
3. Update COM_PORT below to match your Arduino
4. Run this script

Expected behavior:
- Receives startup message and heartbeats
- Can send manual position commands
- Can request status reports
- Monitor real sensor activity if connected
"""

import serial
import time
import threading

# Configuration
COM_PORT = "COM5"  # Update this to match your Arduino's COM port
BAUD_RATE = 9600
TEST_DURATION = 60  # Test for 60 seconds

class IRSensorTester:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.running = False
        self.message_count = 0
        
    def connect(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            print(f"✅ Connected to IR Sensor Arduino on {self.port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect: {e}")
            return False
    
    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("🔌 Disconnected from Arduino")
    
    def send_command(self, command):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write((command + '\n').encode())
            print(f"📤 Sent: {command}")
    
    def read_messages(self):
        """Background thread to read incoming messages"""
        while self.running and self.serial_conn and self.serial_conn.is_open:
            try:
                if self.serial_conn.in_waiting > 0:
                    message = self.serial_conn.readline().decode().strip()
                    if message:
                        self.message_count += 1
                        timestamp = time.strftime("%H:%M:%S")
                        
                        # Categorize messages
                        if message == "IR_MONITOR_READY":
                            print(f"🚀 [{timestamp}] STARTUP: {message}")
                        elif message == "HEARTBEAT":
                            print(f"💓 [{timestamp}] HEARTBEAT")
                        elif message in ["0", "1", "2"]:
                            pos_names = {0: "PASSAGE", 1: "LEFT", 2: "RIGHT"}
                            print(f"📍 [{timestamp}] POSITION: {message} ({pos_names[int(message)]})")
                        elif message.startswith("#"):
                            print(f"💬 [{timestamp}] DEBUG: {message}")
                        elif message.startswith("STATUS"):
                            print(f"📊 [{timestamp}] STATUS: {message}")
                        elif message == "RESET_COMPLETE":
                            print(f"🔄 [{timestamp}] RESET: {message}")
                        elif message == "PONG":
                            print(f"🏓 [{timestamp}] PING RESPONSE: {message}")
                        else:
                            print(f"📨 [{timestamp}] OTHER: {message}")
                            
            except Exception as e:
                if self.running:  # Only print error if we're supposed to be running
                    print(f"❌ Read error: {e}")
                break
            time.sleep(0.01)  # Small delay to prevent excessive CPU usage
    
    def run_interactive_test(self):
        if not self.connect():
            return
        
        self.running = True
        
        # Start background message reader
        reader_thread = threading.Thread(target=self.read_messages, daemon=True)
        reader_thread.start()
        
        print("\n" + "="*60)
        print("IR SENSOR ARDUINO TEST - INTERACTIVE MODE")
        print("="*60)
        print("Available commands:")
        print("  left    - Simulate fish on left side")
        print("  right   - Simulate fish on right side") 
        print("  status  - Request status report")
        print("  reset   - Reset Arduino state")
        print("  ping    - Test communication")
        print("  auto    - Run automatic test sequence")
        print("  quit    - Exit test")
        print("-"*60)
        
        try:
            while self.running:
                command = input("Enter command: ").strip().lower()
                
                if command == "quit":
                    break
                elif command == "auto":
                    self.run_automatic_test()
                elif command in ["left", "right", "status", "reset", "ping"]:
                    self.send_command(command.upper())
                elif command == "":
                    continue
                else:
                    print(f"Unknown command: {command}")
                    
        except KeyboardInterrupt:
            print("\n🛑 Test interrupted by user")
        finally:
            self.running = False
            time.sleep(0.5)  # Give reader thread time to exit
            self.disconnect()
            print(f"📈 Total messages received: {self.message_count}")
    
    def run_automatic_test(self):
        print("\n🤖 Running automatic test sequence...")
        
        # Test sequence
        test_commands = [
            ("PING", 1),
            ("STATUS", 2), 
            ("LEFT", 2),
            ("RIGHT", 2),
            ("RESET", 2),
            ("STATUS", 2)
        ]
        
        for command, wait_time in test_commands:
            print(f"\n🔄 Testing {command}...")
            self.send_command(command)
            time.sleep(wait_time)
        
        print("✅ Automatic test sequence complete!")

def main():
    print("IR Sensor Monitor Arduino Tester")
    print("="*40)
    
    # Check if user wants to change COM port
    port = input(f"Enter COM port (default: {COM_PORT}): ").strip()
    if not port:
        port = COM_PORT
    
    tester = IRSensorTester(port, BAUD_RATE)
    tester.run_interactive_test()

if __name__ == "__main__":
    main()