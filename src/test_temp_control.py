"""
Simple test script for Temperature Control Arduino
Tests relay control commands and safety features

Usage:
1. Upload 03_temperature_control.ino to Arduino
2. Connect Arduino to PC via USB
3. Update COM_PORT below to match your Arduino
4. Run this script

Expected behavior:
- Receives startup message and heartbeats
- Can control individual relays
- Tests safety features (mutual exclusion, emergency stop)
- Status LED should indicate relay states
"""

import serial
import time
import threading

# Configuration
COM_PORT = "COM5"  # Update this to match your Arduino's COM port
BAUD_RATE = 9600

class TempControlTester:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.running = False
        self.message_count = 0
        self.relay_states = {
            "HEAT": False,
            "COOL": False, 
            "BHEAT": False,
            "BCOOL": False
        }
        
    def connect(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            print(f"✅ Connected to Temperature Control Arduino on {self.port}")
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
    
    def map_command(self, user_command):
        """Map user-friendly commands to Arduino commands"""
        command_map = {
            "heat_on": "HEAT_ON",
            "heat_off": "HEAT_OFF",
            "cool_on": "COOL_ON", 
            "cool_off": "COOL_OFF",
            "buffer_heat_on": "BUFFER_HEAT_ON",
            "buffer_heat_off": "BUFFER_HEAT_OFF",
            "buffer_cool_on": "BUFFER_COOL_ON",
            "buffer_cool_off": "BUFFER_COOL_OFF",
            "all_off": "ALL_OFF",
            "safety_override": "SAFETY_OVERRIDE",
            "safety_normal": "SAFETY_NORMAL",
            "status": "STATUS",
            "reset": "RESET",
            "ping": "PING"
        }
        return command_map.get(user_command, user_command.upper())
    
    def read_messages(self):
        """Background thread to read incoming messages"""
        while self.running and self.serial_conn and self.serial_conn.is_open:
            try:
                if self.serial_conn.in_waiting > 0:
                    message = self.serial_conn.readline().decode().strip()
                    if message:
                        self.message_count += 1
                        timestamp = time.strftime("%H:%M:%S")
                        
                        # Parse and categorize messages
                        if message == "TEMP_CTRL_READY":
                            print(f"🚀 [{timestamp}] STARTUP: {message}")
                        elif message == "HEARTBEAT":
                            print(f"💓 [{timestamp}] HEARTBEAT")
                        elif message.startswith("RELAY_"):
                            self.parse_relay_status(message, timestamp)
                        elif message.startswith("ERROR:"):
                            print(f"⚠️  [{timestamp}] ERROR: {message}")
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
                if self.running:
                    print(f"❌ Read error: {e}")
                break
            time.sleep(0.01)
    
    def parse_relay_status(self, message, timestamp):
        """Parse relay status messages and update internal state"""
        try:
            parts = message.split(":")
            if len(parts) == 2:
                relay_type = parts[0].replace("RELAY_", "")
                state = parts[1] == "1"
                
                if relay_type in self.relay_states:
                    self.relay_states[relay_type] = state
                    state_emoji = "🔥" if state else "❄️"
                    state_text = "ON" if state else "OFF"
                    
                    relay_names = {
                        "HEAT": "Heating",
                        "COOL": "Cooling", 
                        "BHEAT": "Buffer Heating",
                        "BCOOL": "Buffer Cooling"
                    }
                    
                    relay_name = relay_names.get(relay_type, relay_type)
                    print(f"{state_emoji} [{timestamp}] {relay_name}: {state_text}")
                    
                    # Show current relay summary
                    active_relays = [name for name, state in self.relay_states.items() if state]
                    if active_relays:
                        print(f"   🔧 Active relays: {', '.join(active_relays)}")
                    else:
                        print(f"   ⚫ All relays OFF")
                        
        except Exception as e:
            print(f"❌ Error parsing relay status: {e}")
    
    def run_interactive_test(self):
        if not self.connect():
            return
        
        self.running = True
        
        # Start background message reader
        reader_thread = threading.Thread(target=self.read_messages, daemon=True)
        reader_thread.start()
        
        print("\n" + "="*60)
        print("TEMPERATURE CONTROL ARDUINO TEST - INTERACTIVE MODE")
        print("="*60)
        print("Relay commands:")
        print("  heat_on / heat_off           - Main heating relay")
        print("  cool_on / cool_off           - Main cooling relay")
        print("  buffer_heat_on / buffer_heat_off - Buffer heating relay")
        print("  buffer_cool_on / buffer_cool_off - Buffer cooling relay")
        print("\nSafety commands:")
        print("  all_off                  - Emergency stop (all relays off)")
        print("  safety_override          - Enable safety override")
        print("  safety_normal            - Disable safety override")
        print("\nStatus commands:")
        print("  status                   - Request status report")
        print("  reset                    - Reset Arduino")
        print("  ping                     - Test communication")
        print("\nTest sequences:")
        print("  safety_test              - Test safety features")
        print("  relay_test               - Test all relays individually")
        print("  quit                     - Exit test")
        print("-"*60)
        
        try:
            while self.running:
                command = input("Enter command: ").strip().lower()
                
                if command == "quit":
                    break
                elif command == "safety_test":
                    self.run_safety_test()
                elif command == "relay_test":
                    self.run_relay_test()
                elif command in ["heat_on", "heat_off", "cool_on", "cool_off",
                               "buffer_heat_on", "buffer_heat_off", "buffer_cool_on", "buffer_cool_off",
                               "all_off", "safety_override", "safety_normal",
                               "status", "reset", "ping"]:
                    arduino_command = self.map_command(command)
                    self.send_command(arduino_command)
                elif command == "":
                    continue
                else:
                    print(f"Unknown command: {command}")
                    print(f"Debug: command length={len(command)}, repr={repr(command)}")
                    valid_commands = ["heat_on", "heat_off", "cool_on", "cool_off",
                                    "buffer_heat_on", "buffer_heat_off", "buffer_cool_on", "buffer_cool_off",
                                    "all_off", "safety_override", "safety_normal",
                                    "status", "reset", "ping"]
                    print(f"Valid commands: {valid_commands}")
                    if command in valid_commands:
                        print("Command IS in valid list - this shouldn't happen!")
                    else:
                        print("Command NOT in valid list")
                    
        except KeyboardInterrupt:
            print("\n🛑 Test interrupted by user")
        finally:
            self.running = False
            # Send all_off for safety
            if self.serial_conn and self.serial_conn.is_open:
                self.send_command("ALL_OFF")
                time.sleep(1)
            time.sleep(0.5)
            self.disconnect()
            print(f"📈 Total messages received: {self.message_count}")
    
    def run_safety_test(self):
        print("\n🛡️ Running safety feature test...")
        
        # Test mutual exclusion
        print("\n1️⃣ Testing mutual exclusion (heating vs cooling)...")
        self.send_command("HEAT_ON")
        time.sleep(1)
        print("   Now trying to enable cooling (should fail)...")
        self.send_command("COOL_ON")
        time.sleep(2)
        
        self.send_command("ALL_OFF")
        time.sleep(1)
        
        # Test reverse mutual exclusion
        print("\n2️⃣ Testing reverse mutual exclusion (cooling vs heating)...")
        self.send_command("COOL_ON")
        time.sleep(1)
        print("   Now trying to enable heating (should fail)...")
        self.send_command("HEAT_ON")
        time.sleep(2)
        
        self.send_command("ALL_OFF")
        time.sleep(1)
        
        # Test buffer mutual exclusion
        print("\n3️⃣ Testing buffer mutual exclusion (buffer heat vs buffer cool)...")
        self.send_command("BUFFER_HEAT_ON")
        time.sleep(1)
        print("   Now trying to enable buffer cooling (should fail)...")
        self.send_command("BUFFER_COOL_ON")
        time.sleep(2)
        
        self.send_command("ALL_OFF")
        time.sleep(1)
        
        # Test reverse buffer mutual exclusion
        print("\n4️⃣ Testing reverse buffer mutual exclusion (buffer cool vs buffer heat)...")
        self.send_command("BUFFER_COOL_ON")
        time.sleep(1)
        print("   Now trying to enable buffer heating (should fail)...")
        self.send_command("BUFFER_HEAT_ON")
        time.sleep(2)
        
        # Test emergency stop
        print("\n5️⃣ Testing emergency stop...")
        self.send_command("HEAT_ON")
        time.sleep(1)
        self.send_command("BUFFER_HEAT_ON")
        time.sleep(1)
        print("   Activating emergency stop...")
        self.send_command("ALL_OFF")
        time.sleep(2)
        
        print("✅ Safety test complete!")
    
    def run_relay_test(self):
        print("\n🔧 Running individual relay test...")
        
        relays = [
            ("HEAT_ON", "HEAT_OFF", "Heating"),
            ("COOL_ON", "COOL_OFF", "Cooling"),
            ("BUFFER_HEAT_ON", "BUFFER_HEAT_OFF", "Buffer Heating"),
            ("BUFFER_COOL_ON", "BUFFER_COOL_OFF", "Buffer Cooling")
        ]
        
        for on_cmd, off_cmd, name in relays:
            print(f"\n🔄 Testing {name} relay...")
            print(f"   Turning ON...")
            self.send_command(on_cmd)
            time.sleep(2)
            print(f"   Turning OFF...")
            self.send_command(off_cmd)
            time.sleep(1)
        
        print("✅ Relay test complete!")

def main():
    print("Temperature Control Arduino Tester")
    print("="*40)
    
    # Check if user wants to change COM port
    port = input(f"Enter COM port (default: {COM_PORT}): ").strip()
    if not port:
        port = COM_PORT
    
    print("\n⚠️  WARNING: Make sure relays are properly wired before testing!")
    print("⚠️  This test will activate real relays - ensure safe conditions!")
    confirm = input("Continue? (y/N): ").strip().lower()
    
    if confirm != 'y':
        print("Test cancelled.")
        return
    
    tester = TempControlTester(port, BAUD_RATE)
    tester.run_interactive_test()

if __name__ == "__main__":
    main()