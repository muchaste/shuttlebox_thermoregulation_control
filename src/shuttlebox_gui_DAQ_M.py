"""
Shuttlebox Thermoregulation Control GUI
Integrates TC-08 temperature monitoring, IR sensor monitoring, and temperature control
for automated behavioral thermoregulation experiments

Author: Shuttlebox Control System
"""

import sys
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from datetime import datetime, timedelta
import threading
import time
import serial
import serial.tools.list_ports
import csv
import json
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import queue
import nidaqmx
from nidaqmx.constants import LineGrouping

# Import our controllers (assuming they're in the same directory)
try:
    from tc08_controller import TC08Controller, TC08Config
except ImportError:
    print("Warning: tc08_controller not found. Temperature monitoring disabled.")
    TC08Controller = None

@dataclass
class SystemConfig:
    """Configuration for the entire shuttlebox system"""
    tc08_channels: List[int]
    tc08_sample_interval_ms: int
    ir_sensor_com_port: str
    daq_device_name: str
    data_log_path: str
    channel_names: Dict[int, str]

class SerialDevice:
    """Base class for Arduino serial communication"""
    def __init__(self, port: str, baud_rate: int = 9600):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.connected = False
        self.message_queue = queue.Queue()
        self.reader_thread = None
        self.running = False
        
        # Heartbeat monitoring
        self.last_heartbeat = None
        self.heartbeat_timeout = 15  # 15 seconds (3 missed heartbeats)
        self.connection_lost = False
    
    def connect(self, expected_startup_msg: str = None) -> bool:
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2)  # Arduino initialization
            
            # If startup message validation is required
            if expected_startup_msg:
                startup_received = False
                timeout_start = time.time()
                
                while time.time() - timeout_start < 5:  # 5 second timeout
                    if self.serial_conn.in_waiting > 0:
                        try:
                            line = self.serial_conn.readline().decode().strip()
                            if line == expected_startup_msg:
                                startup_received = True
                                break
                        except:
                            continue
                    time.sleep(0.1)
                
                if not startup_received:
                    self.serial_conn.close()
                    return False
            
            self.connected = True
            self.running = True
            self.reader_thread = threading.Thread(target=self._read_messages, daemon=True)
            self.reader_thread.start()
            return True
        except Exception as e:
            print(f"Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        self.running = False
        self.connected = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
    
    def send_command(self, command: str):
        if self.connected and self.serial_conn:
            self.serial_conn.write((command + '\n').encode())
    
    def _read_messages(self):
        while self.running and self.serial_conn and self.serial_conn.is_open:
            try:
                if self.serial_conn.in_waiting > 0:
                    message = self.serial_conn.readline().decode().strip()
                    if message:
                        # Handle heartbeat messages
                        if message == "HEARTBEAT":
                            self.last_heartbeat = time.time()
                            self.connection_lost = False
                        else:
                            self.message_queue.put((datetime.now(), message))
            except Exception as e:
                if self.running:
                    print(f"Serial read error: {e}")
                break
            time.sleep(0.01)
    
    def is_connection_healthy(self) -> bool:
        """Check if connection is healthy based on heartbeat"""
        if self.last_heartbeat is None:
            return self.connected  # No heartbeat received yet, use basic connection status
        
        time_since_heartbeat = time.time() - self.last_heartbeat
        if time_since_heartbeat > self.heartbeat_timeout:
            self.connection_lost = True
            return False
        return self.connected and not self.connection_lost

class IRSensorController(SerialDevice):
    """Controller for IR sensor monitoring Arduino"""
    def __init__(self, port: str):
        super().__init__(port)
        self.fish_position = 0  # 0=passage, 1=left, 2=right
        self.sensor_states = [False] * 10  # 10 IR sensors
        self.position_history = []
    
    def get_latest_data(self):
        """Get latest IR sensor data from message queue"""
        while not self.message_queue.empty():
            timestamp, message = self.message_queue.get()
            self._process_message(message, timestamp)
        
        return {
            'fish_position': self.fish_position,
            'sensor_states': self.sensor_states.copy(),
            'position_history': self.position_history[-100:].copy()  # Last 100 positions
        }
    
    def override_fish_position(self, position: int):
        """Override the current fish position (for manual corrections)"""
        if position in [0, 1, 2]:
            self.fish_position = position
            # Add to position history as a manual correction
            self.position_history.append({
                'timestamp': datetime.now(),
                'position': position,
                'manual_correction': True
            })
    
    def _process_message(self, message: str, timestamp: datetime):
        message = message.strip()
        
        if message.startswith("POSITION:"):
            # New format: "POSITION:0", "POSITION:1", "POSITION:2"
            try:
                position = int(message.split(":")[1])
                if position in [0, 1, 2]:
                    self.fish_position = position
                    self.position_history.append({
                        'timestamp': timestamp,
                        'position': self.fish_position
                    })
            except (ValueError, IndexError):
                pass
        elif message in ["0", "1", "2"]:
            # Legacy format: just the number
            self.fish_position = int(message)
            self.position_history.append({
                'timestamp': timestamp,
                'position': self.fish_position
            })
        elif message.startswith("SENSORS:"):
            # Format: "SENSORS:1010101010" (10 sensors: 5 left + 5 right)
            # 1=clear/not interrupted, 0=interrupted
            try:
                sensor_data = message.split(":")[1]
                if len(sensor_data) >= 10:
                    for i, state in enumerate(sensor_data[:10]):
                        # Convert to boolean: 1=clear (True), 0=interrupted (False)
                        self.sensor_states[i] = (state == '1')
            except IndexError:
                pass
        elif message == "HEARTBEAT":
            # Update heartbeat timestamp for connection health monitoring
            self.last_heartbeat = time.time()
            self.connection_lost = False

class DAQTempController:
    """Controller for NI DAQ temperature control (replaces Arduino)"""
    def __init__(self, device_name: str = "Dev1"):
        self.device_name = device_name
        self.connected = False
        self.tasks = {}
        
        # Map relay names to DAQ port lines
        self.line_mapping = {
            "HEAT": 0,      # Port0/Line0
            "COOL": 1,      # Port0/Line1
            "BHEAT": 2,     # Port0/Line2
            "BCOOL": 3      # Port0/Line3
        }
        
        self.relay_states = {
            "HEAT": False,
            "COOL": False,
            "BHEAT": False,
            "BCOOL": False
        }
        
        # Track desired states (for UI consistency)
        self.desired_states = {
            "HEAT": False,
            "COOL": False,
            "BHEAT": False,
            "BCOOL": False
        }
        
        self.last_command_time = {}
        self.command_timeout = 0.5  # DAQ is faster, shorter timeout
    
    def connect(self) -> bool:
        """Initialize DAQ tasks for all 4 digital output lines"""
        try:
            # Create individual tasks for each pump/relay
            for relay_name, line_num in self.line_mapping.items():
                task = nidaqmx.Task(f"task_{relay_name}")
                channel_name = f"{self.device_name}/port0/line{line_num}"
                task.do_channels.add_do_chan(channel_name, line_grouping=LineGrouping.CHAN_PER_LINE)
                task.start()
                self.tasks[relay_name] = task
            
            self.connected = True
            print(f"DAQ temperature controller connected: {self.device_name}")
            return True
            
        except Exception as e:
            print(f"Failed to connect to DAQ device {self.device_name}: {e}")
            self.disconnect()
            return False
    
    def disconnect(self):
        """Close all DAQ tasks and cleanup"""
        try:
            # Stop and close all tasks
            for relay_name, task in self.tasks.items():
                try:
                    task.stop()
                    task.close()
                except:
                    pass
            
            self.tasks.clear()
            self.connected = False
            print("DAQ temperature controller disconnected")
            
        except Exception as e:
            print(f"Error during DAQ disconnect: {e}")
    
    def _write_digital_line(self, relay_name: str, state: bool):
        """Write digital output to specific DAQ line"""
        try:
            if relay_name in self.tasks:
                self.tasks[relay_name].write(state)
                self.relay_states[relay_name] = state
                self.desired_states[relay_name] = state
                self.last_command_time[relay_name] = time.time()
                return True
            else:
                print(f"Task for {relay_name} not found")
                return False
        except Exception as e:
            print(f"Error writing to DAQ line {relay_name}: {e}")
            return False
    
    def set_heating(self, enable: bool):
        """Set heating relay state"""
        self._write_digital_line("HEAT", enable)
    
    def set_cooling(self, enable: bool):
        """Set cooling relay state"""
        self._write_digital_line("COOL", enable)
    
    def set_buffer_heating(self, enable: bool):
        """Set buffer heating relay state"""
        self._write_digital_line("BHEAT", enable)
    
    def set_buffer_cooling(self, enable: bool):
        """Set buffer cooling relay state"""
        self._write_digital_line("BCOOL", enable)
    
    def emergency_stop(self):
        """Emergency stop all relays"""
        for relay_name in self.relay_states.keys():
            self._write_digital_line(relay_name, False)
    
    def get_status(self):
        """Get current relay status (DAQ writes are immediate)"""
        return self.relay_states.copy()
    
    def get_relay_sync_status(self):
        """Check if actual relay states match desired states (always synced for DAQ)"""
        sync_status = {}
        current_time = time.time()
        
        for relay in self.relay_states:
            # DAQ writes are immediate and reliable
            actual = self.relay_states[relay]
            desired = self.desired_states[relay]
            last_cmd_time = self.last_command_time.get(relay, 0)
            time_since_cmd = current_time - last_cmd_time
            
            sync_status[relay] = {
                'actual': actual,
                'desired': desired,
                'synced': True,  # DAQ is always synced
                'timeout': False,  # DAQ doesn't timeout
                'time_since_command': time_since_cmd
            }
        
        return sync_status
    
    def retry_failed_commands(self):
        """No retry needed for DAQ - writes are immediate"""
        pass
    
    def keep_alive(self):
        """No keep-alive needed for DAQ"""
        pass
    
    def is_connection_healthy(self) -> bool:
        """Check if DAQ connection is healthy"""
        return self.connected and len(self.tasks) == 4

class ShuttleboxGUI:
    """Main GUI application for shuttlebox thermoregulation control"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Shuttlebox Thermoregulation Control System")
        self.root.geometry("1400x900")
        
        # System components
        self.tc08_controller = None
        self.ir_sensor_controller = None
        self.temp_control_controller = None
        
        # Configuration
        self.config = SystemConfig(
            tc08_channels=[1, 2, 3, 4, 5, 6],
            tc08_sample_interval_ms=1000,
            ir_sensor_com_port="COM5",
            daq_device_name="Dev1",
            data_log_path="shuttlebox_data",
            channel_names={
                1: "Warm Inlet", 2: "Warm Center", 3: "Warm Buffer",
                4: "Cool Inlet", 5: "Cool Center", 6: "Cool Buffer"
            }
        )
        
        # Data storage
        self.temperature_data = {ch: [] for ch in self.config.tc08_channels}
        self.temperature_timestamps = {ch: [] for ch in self.config.tc08_channels}
        
        # Average temperature storage
        self.warm_avg_data = []
        self.cold_avg_data = []
        self.avg_timestamps = []
        
        # Fish temperature storage (based on position)
        self.fish_temp_data = []
        self.fish_temp_timestamps = []
        
        # Dedicated recording data storage (for trial recording)
        self.recording_data = []  # List of dictionaries for each sample
        self.recording_filename = None
        self.recording_metadata_filename = None
        self.flush_interval = 300  # Flush to file every 5 minutes (300 seconds)
        self.last_flush_time = 0
        
        # GUI update timer
        self.update_timer = None
        self.monitoring_active = False  # For data recording (IR + temp + all systems)
        self.plotting_active = False    # For temperature plotting (TC-08 only)
        self.recording_active = False   # For data recording to CSV
        
        # Temperature control modes
        self.static_control_active = False
        self.dynamic_control_active = False
        self.difference_control_active = False  # Active difference control in dynamic mode
        self.last_control_action = time.time()  # For hysteresis timing
        
        # Manual position control
        self.manual_position_override = None  # Track manual overrides
        self.persistent_manual_mode = False   # Track persistent manual mode
        
        # Connection health monitoring
        self._ir_warning_shown = False
        self._temp_warning_shown = False
        
        self.create_widgets()
        self.refresh_com_ports()  # Initialize COM port lists
        self.update_monitoring_button_state()  # Set initial button state
        self.start_gui_updates()
    
    def create_widgets(self):
        """Create the main GUI layout"""
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left panel (configuration and control)
        left_frame = ttk.Frame(main_frame, width=400)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left_frame.pack_propagate(False)
        
        # Right panel (monitoring and visualization)
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.create_left_panel(left_frame)
        self.create_right_panel(right_frame)
    
    def create_left_panel(self, parent):
        """Create configuration and control panels"""
        
        # TC-08 Configuration Section
        tc08_frame = ttk.LabelFrame(parent, text="TC-08 Temperature Sensor Configuration", padding="10")
        tc08_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Channel configuration header
        ttk.Label(tc08_frame, text="Channel Configuration:", font=('Arial', 9, 'bold')).grid(row=0, column=0, columnspan=3, sticky=tk.W, pady=(0, 5))
        ttk.Label(tc08_frame, text="Device Ch#", font=('Arial', 8)).grid(row=1, column=1, padx=(20, 5), pady=2)
        ttk.Label(tc08_frame, text="Channel ID", font=('Arial', 8)).grid(row=1, column=2, padx=5, pady=2)
        ttk.Label(tc08_frame, text="Enable", font=('Arial', 8)).grid(row=1, column=3, padx=5, pady=2)
        
        # Channel configuration rows
        self.channel_vars = {}
        self.channel_id_vars = {}
        self.channel_enable_vars = {}
        
        default_channels = [6, 7, 5, 3, 2, 4]
        default_ids = ["w_inlet", "w_front", "w_back", "c_inlet", "c_front", "c_back"]
        
        current_row = 2
        
        # Add WARM SIDE label
        warm_label = ttk.Label(tc08_frame, text="WARM SIDE", font=("Arial", 9, "bold"), foreground="darkred")
        warm_label.grid(row=current_row, column=0, columnspan=3, sticky=tk.W, pady=(5, 0))
        # current_row += 1
        
        # Add warm side channels (0-2)
        for i in range(3):
            channel = default_channels[i]
            channel_id = default_ids[i]
            
            # Channel number (device channel)
            self.channel_vars[i] = tk.StringVar(value=str(channel))
            channel_entry = ttk.Entry(tc08_frame, textvariable=self.channel_vars[i], width=8, justify=tk.CENTER)
            channel_entry.grid(row=current_row, column=1, padx=(20, 5), pady=1)
            
            # Channel ID (user-defined name)
            self.channel_id_vars[i] = tk.StringVar(value=channel_id)
            id_entry = ttk.Entry(tc08_frame, textvariable=self.channel_id_vars[i], width=15)
            id_entry.grid(row=current_row, column=2, padx=5, pady=1)
            
            # Enable checkbox
            self.channel_enable_vars[i] = tk.BooleanVar(value=True)
            enable_check = ttk.Checkbutton(tc08_frame, variable=self.channel_enable_vars[i])
            enable_check.grid(row=current_row, column=3, padx=5, pady=1)
            
            current_row += 1
        
        # Add COLD SIDE label
        cold_label = ttk.Label(tc08_frame, text="COLD SIDE", font=("Arial", 9, "bold"), foreground="darkblue")
        cold_label.grid(row=current_row, column=0, columnspan=3, sticky=tk.W, pady=(10, 0))
        # current_row += 1
        
        # Add cold side channels (3-5)
        for i in range(3, 6):
            channel = default_channels[i]
            channel_id = default_ids[i]
            
            # Channel number (device channel)
            self.channel_vars[i] = tk.StringVar(value=str(channel))
            channel_entry = ttk.Entry(tc08_frame, textvariable=self.channel_vars[i], width=8, justify=tk.CENTER)
            channel_entry.grid(row=current_row, column=1, padx=(20, 5), pady=1)
            
            # Channel ID (user-defined name)
            self.channel_id_vars[i] = tk.StringVar(value=channel_id)
            id_entry = ttk.Entry(tc08_frame, textvariable=self.channel_id_vars[i], width=15)
            id_entry.grid(row=current_row, column=2, padx=5, pady=1)
            
            # Enable checkbox
            self.channel_enable_vars[i] = tk.BooleanVar(value=True)
            enable_check = ttk.Checkbutton(tc08_frame, variable=self.channel_enable_vars[i])
            enable_check.grid(row=current_row, column=3, padx=5, pady=1)
            
            current_row += 1
        
        # Sample interval
        ttk.Label(tc08_frame, text="Sample Interval (ms):").grid(row=current_row, column=0, sticky=tk.W, pady=(10, 2))
        self.tc08_interval_var = tk.StringVar(value="1000")
        ttk.Entry(tc08_frame, textvariable=self.tc08_interval_var, width=15).grid(row=current_row, column=1, padx=(0, 5), pady=(10, 2), sticky=tk.W)
        current_row += 1
        
        # Plot period
        ttk.Label(tc08_frame, text="Plot Period (s):").grid(row=current_row, column=0, sticky=tk.W, pady=2)
        self.plot_period_var = tk.StringVar(value="300")
        ttk.Entry(tc08_frame, textvariable=self.plot_period_var, width=15).grid(row=current_row, column=1, padx=(0, 5), pady=2, sticky=tk.W)
        # current_row += 1
        
        # TC-08 connect button
        self.tc08_connect_btn = ttk.Button(tc08_frame, text="Connect TC-08", command=self.connect_tc08)
        self.tc08_connect_btn.grid(row=current_row, column=2, columnspan=1, pady=10)
        
        # TC-08 status indicator (circle)
        self.tc08_status_canvas = tk.Canvas(tc08_frame, width=20, height=20, highlightthickness=0)
        self.tc08_status_canvas.grid(row=current_row, column=3, pady=5)
        self.tc08_status_circle = self.tc08_status_canvas.create_oval(2, 2, 18, 18, fill="red", outline="black")
        
        # Arduino Configuration Section
        arduino_frame = ttk.LabelFrame(parent, text="Arduino Configuration", padding="10")
        arduino_frame.pack(fill=tk.X, pady=(0, 10))
        current_row = 0

        # Refresh COM ports and DAQ devices button
        refresh_btn = ttk.Button(arduino_frame, text="Refresh Ports & Devices", command=self.refresh_com_ports)
        refresh_btn.grid(row=current_row, column=0, columnspan=2, pady=(0, 10), sticky=tk.W)
        current_row += 1

        # IR Sensor Arduino
        ttk.Label(arduino_frame, text="IR Sensor COM Port:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.ir_com_var = tk.StringVar()
        self.ir_com_combo = ttk.Combobox(arduino_frame, textvariable=self.ir_com_var, width=12, state="readonly")
        self.ir_com_combo.grid(row=current_row, column=1, padx=(10, 0))
        self.ir_connect_btn = ttk.Button(arduino_frame, text="Connect", command=self.connect_ir_sensor)
        self.ir_connect_btn.grid(row=current_row, column=2, padx=(10, 0))

        # IR status indicator (circle)
        self.ir_status_canvas = tk.Canvas(arduino_frame, width=20, height=20, highlightthickness=0)
        self.ir_status_canvas.grid(row=current_row, column=3, padx=(10, 0))
        self.ir_status_circle = self.ir_status_canvas.create_oval(2, 2, 18, 18, fill="red", outline="black")
        current_row += 1
        
        # Temperature Control DAQ Device
        ttk.Label(arduino_frame, text="DAQ Device:").grid(row=current_row, column=0, sticky=tk.W, pady=2)
        self.daq_device_var = tk.StringVar(value="Dev1")
        self.daq_device_combo = ttk.Combobox(arduino_frame, textvariable=self.daq_device_var, width=12, state="readonly")
        self.daq_device_combo.grid(row=current_row, column=1, padx=(10, 0))
        self.temp_connect_btn = ttk.Button(arduino_frame, text="Connect", command=self.connect_temp_control)
        self.temp_connect_btn.grid(row=current_row, column=2, padx=(10, 0))
        
        # Temperature control status indicator (circle)
        self.temp_status_canvas = tk.Canvas(arduino_frame, width=20, height=20, highlightthickness=0)
        self.temp_status_canvas.grid(row=current_row, column=3, padx=(10, 0))
        self.temp_status_circle = self.temp_status_canvas.create_oval(2, 2, 18, 18, fill="red", outline="black")
        
        # Control Section
        control_frame = ttk.LabelFrame(parent, text="System Control", padding="10")
        control_frame.pack(fill=tk.X, pady=(0, 10))
        current_row = 0

        # Temperature Control Configuration
        ttk.Label(control_frame, text="Automated Temperature Control:", font=('Arial', 9, 'bold')).pack()
        
        # Control parameters frame
        control_params_frame = ttk.Frame(control_frame)
        control_params_frame.pack(fill=tk.X, pady=5)
        
        # Start temperature
        ttk.Label(control_params_frame, text="Start Temperature (°C):").grid(row=current_row, column=0, sticky=tk.W, pady=2)
        self.start_temp_var = tk.StringVar(value="25.0")
        ttk.Entry(control_params_frame, textvariable=self.start_temp_var, width=8).grid(row=current_row, column=1, padx=5, pady=2)

        # Hysteresis
        ttk.Label(control_params_frame, text="Hysteresis (°C):").grid(row=current_row, column=2, sticky=tk.W, padx=(20, 5), pady=2)
        self.hysteresis_var = tk.StringVar(value="0.2")
        ttk.Entry(control_params_frame, textvariable=self.hysteresis_var, width=8).grid(row=current_row, column=3, padx=5, pady=2)
        current_row += 1

        # Temperature difference
        ttk.Label(control_params_frame, text="Temp Difference (°C):").grid(row=current_row, column=0, sticky=tk.W, pady=2)
        self.temp_diff_var = tk.StringVar(value="2.0")
        ttk.Entry(control_params_frame, textvariable=self.temp_diff_var, width=8).grid(row=current_row, column=1, padx=5, pady=2)

        # Start side selection
        ttk.Label(control_params_frame, text="Start Side:").grid(row=current_row, column=2, sticky=tk.W, padx=(20, 5), pady=2)
        self.start_side_var = tk.StringVar(value="warm")
        start_side_combo = ttk.Combobox(control_params_frame, textvariable=self.start_side_var, 
                                       values=["warm", "cold"], state="readonly", width=6)
        start_side_combo.grid(row=current_row, column=3, padx=5, pady=2)
        current_row += 1

        # Control mode buttons
        control_mode_frame = ttk.Frame(control_frame)
        control_mode_frame.pack(fill=tk.X, pady=5)
        
        self.static_control_btn = ttk.Button(control_mode_frame, text="Start Static Control", 
                                            command=self.toggle_static_control, state=tk.DISABLED)
        self.static_control_btn.grid(row=0, column=0, padx=2, pady=5)
        
        self.dynamic_control_btn = ttk.Button(control_mode_frame, text="Start Dynamic Control", 
                                             command=self.toggle_dynamic_control, state=tk.DISABLED)
        self.dynamic_control_btn.grid(row=0, column=1, padx=2, pady=5)
        
        self.difference_control_btn = ttk.Button(control_mode_frame, text="Difference Control", 
                                               command=self.toggle_difference_control, state=tk.DISABLED)
        self.difference_control_btn.grid(row=0, column=2, padx=2, pady=5)
        
        # Control status
        self.control_status_var = tk.StringVar(value="Manual Mode")
        ttk.Label(control_frame, textvariable=self.control_status_var, font=('Arial', 9, 'italic')).pack(pady=2)
        
        # Relay status indicators
        relay_status_frame = ttk.LabelFrame(control_frame, text="Relay Status", padding="5")
        relay_status_frame.pack(fill=tk.X, pady=5)
        
        # Create relay status indicators in 2x2 grid
        relay_grid = ttk.Frame(relay_status_frame)
        relay_grid.pack()
        current_row = 0

        # Warm side relays (top row)
        warm_relay_frame = ttk.Frame(relay_grid)
        warm_relay_frame.grid(row=current_row, column=0, columnspan=2, pady=2)
        
        ttk.Label(warm_relay_frame, text="Heat:", font=('Arial', 8)).pack(side=tk.LEFT, padx=2)
        self.heat_status_canvas = tk.Canvas(warm_relay_frame, width=16, height=16, highlightthickness=0)
        self.heat_status_canvas.pack(side=tk.LEFT, padx=2)
        self.heat_status_circle = self.heat_status_canvas.create_oval(2, 2, 14, 14, fill="gray", outline="black")
        
        ttk.Label(warm_relay_frame, text="Buffer Heat:", font=('Arial', 8)).pack(side=tk.LEFT, padx=(15, 2))
        self.bheat_status_canvas = tk.Canvas(warm_relay_frame, width=16, height=16, highlightthickness=0)
        self.bheat_status_canvas.pack(side=tk.LEFT, padx=2)
        self.bheat_status_circle = self.bheat_status_canvas.create_oval(2, 2, 14, 14, fill="gray", outline="black")
        current_row += 1

        # Cool side relays (bottom row)
        cool_relay_frame = ttk.Frame(relay_grid)
        cool_relay_frame.grid(row=current_row, column=0, columnspan=2, pady=2)
        
        ttk.Label(cool_relay_frame, text="Cool:", font=('Arial', 8)).pack(side=tk.LEFT, padx=2)
        self.cool_status_canvas = tk.Canvas(cool_relay_frame, width=16, height=16, highlightthickness=0)
        self.cool_status_canvas.pack(side=tk.LEFT, padx=2)
        self.cool_status_circle = self.cool_status_canvas.create_oval(2, 2, 14, 14, fill="gray", outline="black")
        
        ttk.Label(cool_relay_frame, text="Buffer Cool:", font=('Arial', 8)).pack(side=tk.LEFT, padx=(15, 2))
        self.bcool_status_canvas = tk.Canvas(cool_relay_frame, width=16, height=16, highlightthickness=0)
        self.bcool_status_canvas.pack(side=tk.LEFT, padx=2)
        self.bcool_status_circle = self.bcool_status_canvas.create_oval(2, 2, 14, 14, fill="gray", outline="black")
        
        # Manual temperature control buttons
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(control_frame, text="Manual Temperature Control:", font=('Arial', 9, 'bold')).pack()
        
        temp_btn_frame = ttk.Frame(control_frame)
        temp_btn_frame.pack(fill=tk.X, pady=5)
        current_row = 0

        self.heat_btn = ttk.Button(temp_btn_frame, text="Heat ON", command=lambda: self.manual_temp_control('heat', True))
        self.heat_btn.grid(row=current_row, column=0, padx=2)

        self.heat_off_btn = ttk.Button(temp_btn_frame, text="Heat OFF", command=lambda: self.manual_temp_control('heat', False))
        self.heat_off_btn.grid(row=current_row, column=1, padx=2)

        # temp_btn_frame2 = ttk.Frame(control_frame)
        # temp_btn_frame2.pack(fill=tk.X, pady=5)
        
        self.cool_btn = ttk.Button(temp_btn_frame, text="Cool ON", command=lambda: self.manual_temp_control('cool', True))
        self.cool_btn.grid(row=current_row, column=2, padx=2)

        self.cool_off_btn = ttk.Button(temp_btn_frame, text="Cool OFF", command=lambda: self.manual_temp_control('cool', False))
        self.cool_off_btn.grid(row=current_row, column=3, padx=2)

        self.emergency_btn = ttk.Button(control_frame, text="EMERGENCY STOP", command=self.emergency_stop)
        self.emergency_btn.pack(fill=tk.X, pady=10)
        self.emergency_btn.configure(style="Emergency.TButton")
        
        # Data controls
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        # Recording controls at bottom
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(control_frame, text="Data Recording:", font=('Arial', 9, 'bold')).pack()
        
        # Fish ID input
        fish_id_frame = ttk.Frame(control_frame)
        fish_id_frame.pack(fill=tk.X, pady=5)
        ttk.Label(fish_id_frame, text="Fish ID:").pack(side=tk.LEFT)
        self.fish_id_var = tk.StringVar(value="")
        ttk.Entry(fish_id_frame, textvariable=self.fish_id_var, width=15).pack(side=tk.RIGHT, padx=(5, 0))
        
        self.start_recording_btn = ttk.Button(control_frame, text="Start Recording", command=self.start_recording, state=tk.DISABLED)
        self.start_recording_btn.pack(fill=tk.X, pady=2)
        
        self.stop_recording_btn = ttk.Button(control_frame, text="Stop Recording", command=self.stop_recording, state=tk.DISABLED)
        self.stop_recording_btn.pack(fill=tk.X, pady=2)
        
        self.clear_btn = ttk.Button(control_frame, text="Clear Data", command=self.clear_data)
        self.clear_btn.pack(fill=tk.X, pady=2)
    
    def create_right_panel(self, parent):
        """Create monitoring and visualization panels"""
        
        # Temperature plot (top)
        plot_frame = ttk.LabelFrame(parent, text="Temperature Monitoring", padding="10")
        plot_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        self.create_temperature_plot(plot_frame)
        
        # Status visualization (bottom)
        status_frame = ttk.LabelFrame(parent, text="System Status", padding="10")
        status_frame.pack(fill=tk.X)
        
        self.create_status_visualization(status_frame)
    
    def create_temperature_plot(self, parent):
        """Create matplotlib temperature plot"""
        self.fig = Figure(figsize=(10, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        
        # Initialize empty plot - will be configured when TC-08 connects
        self.temp_lines = {}
        
        self.ax.set_xlabel('Time (seconds)')
        self.ax.set_ylabel('Temperature (°C)')
        self.ax.set_title('Real-time Temperature Monitoring')
        self.ax.grid(True, alpha=0.3)
        
        # Add placeholder text
        self.ax.text(0.5, 0.5, 'Connect TC-08 to start temperature plotting', 
                    transform=self.ax.transAxes, ha='center', va='center',
                    fontsize=12, style='italic', color='gray')
        
        self.fig.tight_layout()
        
        # Embed plot in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def update_plot_configuration(self):
        """Update plot configuration for warm/cold side channels with averages"""
        # Clear existing lines
        for line in self.temp_lines.values():
            line.remove()
        self.temp_lines.clear()
        
        # Clear placeholder text
        self.ax.clear()
        self.ax.set_xlabel('Time (seconds)')
        self.ax.set_ylabel('Temperature (°C)')
        self.ax.set_title('Real-time Temperature Monitoring - Warm vs Cold Side')
        self.ax.grid(True, alpha=0.3)
        
        # Define warm and cold channels (first 3 = warm, last 3 = cold)
        warm_channels = [ch for i, ch in enumerate(self.config.tc08_channels) if i < 3]
        cold_channels = [ch for i, ch in enumerate(self.config.tc08_channels) if i >= 3]
        
        # Warm side individual channels (red hues, thin, semi-transparent)
        warm_colors = ['#8B0000', '#B22222', '#DC143C']  # dark red, crimson, red
        for i, channel in enumerate(warm_channels):
            if i < len(warm_colors):
                channel_name = self.config.channel_names.get(channel, f"Ch{channel}")
                line, = self.ax.plot([], [], label=f"W-Ch{channel}: {channel_name}", 
                                   color=warm_colors[i], linewidth=1, alpha=0.6)
                self.temp_lines[channel] = line
        
        # Cold side individual channels (blue hues, thin, semi-transparent)
        cold_colors = ['#000080', '#4169E1', '#87CEEB']  # navy, royal blue, sky blue
        for i, channel in enumerate(cold_channels):
            if i < len(cold_colors):
                channel_name = self.config.channel_names.get(channel, f"Ch{channel}")
                line, = self.ax.plot([], [], label=f"C-Ch{channel}: {channel_name}", 
                                   color=cold_colors[i], linewidth=1, alpha=0.6)
                self.temp_lines[channel] = line
        
        # Average lines (thick, opaque)
        self.warm_avg_line, = self.ax.plot([], [], label='WARM AVERAGE', 
                                          color='red', linewidth=2, alpha=1.0)
        self.cold_avg_line, = self.ax.plot([], [], label='COLD AVERAGE', 
                                          color='blue', linewidth=2, alpha=1.0)
        
        # Fish temperature line (bold, black) - only visible when IR sensor connected
        self.fish_temp_line, = self.ax.plot([], [], label='FISH TEMPERATURE', 
                                           color='black', linewidth=2, alpha=1.0)
        
        # Initialize average data storage
        self.warm_avg_data = []
        self.cold_avg_data = []
        self.avg_timestamps = []
        
        # Initialize fish temperature data storage (synchronized with average data)
        self.fish_temp_data = []
        
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        self.canvas.draw()
        
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        self.fig.tight_layout()
        self.canvas.draw()
    
    def create_status_visualization(self, parent):
        """Create fish position and IR sensor status visualization"""
        
        # Fish position visualization
        fish_frame = ttk.LabelFrame(parent, text="Fish Position")
        fish_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # Current position status
        self.fish_position_var = tk.StringVar(value="Position: Unknown")
        ttk.Label(fish_frame, textvariable=self.fish_position_var, font=('Arial', 10, 'bold')).pack(pady=5)
        
        # Persistent manual control checkbox
        self.persistent_manual_var = tk.BooleanVar(value=False)
        self.persistent_manual_checkbox = ttk.Checkbutton(fish_frame, 
                                                         text="Persistent Manual Control", 
                                                         variable=self.persistent_manual_var,
                                                         command=self.toggle_persistent_manual_control)
        self.persistent_manual_checkbox.pack(pady=2)
        
        position_frame = ttk.Frame(fish_frame)
        position_frame.pack(expand=True, fill=tk.BOTH, padx=10, pady=10)
        
        # Position boxes (now as clickable buttons)
        self.position_buttons = {}
        positions = ['LEFT', 'PASSAGE', 'RIGHT']
        for i, pos in enumerate(positions):
            # Create button with manual position override functionality
            arduino_pos = [1, 0, 2][i]  # Map display order to Arduino codes
            
            button = tk.Button(position_frame, text=pos, font=('Arial', 12, 'bold'),
                             relief=tk.RAISED, borderwidth=2, bg='lightgray',
                             command=lambda p=arduino_pos: self.set_manual_fish_position(p))
            button.grid(row=0, column=i, padx=5, pady=5, sticky="nsew")
            
            # Store button reference with Arduino position code
            self.position_buttons[arduino_pos] = button
            
            position_frame.grid_columnconfigure(i, weight=1)
        
        # IR Sensor visualization
        ir_frame = ttk.LabelFrame(parent, text="IR Sensors")
        ir_frame.pack(side=tk.RIGHT, fill=tk.Y)
        
        sensor_canvas = tk.Canvas(ir_frame, width=120, height=250, bg='white')
        sensor_canvas.pack(padx=10, pady=10)
        
        # Create sensor circles (2 columns of 5)
        self.sensor_circles = {}
        for i in range(10):
            row = i % 5
            col = i // 5
            x = 30 + col * 60
            y = 30 + row * 40
            
            circle = sensor_canvas.create_oval(x-15, y-15, x+15, y+15, 
                                             fill='lightgray', outline='black', width=2)
            label = sensor_canvas.create_text(x, y, text=str(i+1), font=('Arial', 8, 'bold'))
            
            self.sensor_circles[i] = circle
        
        self.sensor_canvas = sensor_canvas
    
    def refresh_com_ports(self):
        """Refresh available COM ports and DAQ devices in dropdown menus"""
        try:
            # Refresh COM ports for IR sensor
            ports = serial.tools.list_ports.comports()
            port_names = [port.device for port in ports]
            
            # Update dropdown values (only IR sensor uses COM port now)
            self.ir_com_combo['values'] = port_names
            
            # Set default values if available
            if port_names:
                if not self.ir_com_var.get():
                    self.ir_com_var.set(port_names[0])
            
            # Refresh DAQ devices
            try:
                daq_system = nidaqmx.system.System()
                daq_devices = daq_system.devices.device_names
                
                if daq_devices:
                    self.daq_device_combo['values'] = daq_devices
                    # Set default if not already set
                    if not self.daq_device_var.get() or self.daq_device_var.get() not in daq_devices:
                        self.daq_device_var.set(daq_devices[0])
                else:
                    self.daq_device_combo['values'] = []
                    messagebox.showwarning("DAQ Warning", 
                                         "No DAQ devices detected. Please check:\n"
                                         "1. DAQ device is connected via USB\n"
                                         "2. NI-DAQmx driver is installed\n"
                                         "3. Device is visible in NI MAX")
            except Exception as daq_error:
                print(f"Warning: Could not list DAQ devices: {daq_error}")
                # Set a default value even if we can't list devices
                if not self.daq_device_var.get():
                    self.daq_device_var.set("Dev1")
                self.daq_device_combo['values'] = ["Dev1", "Dev2", "Dev3"]
                
        except Exception as e:
            messagebox.showerror("Error", f"Failed to refresh ports/devices: {e}")
    
    def connect_tc08(self):
        """Connect to TC-08 temperature sensor"""
        try:
            if TC08Controller is None:
                messagebox.showerror("Error", "TC-08 controller not available")
                return
            
            # Parse channel configuration from the new format
            channels = []
            channel_names = {}
            
            for i in range(6):  # 6 channel configuration rows
                if self.channel_enable_vars[i].get():  # Only include enabled channels
                    try:
                        channel_num = int(self.channel_vars[i].get())
                        channel_id = self.channel_id_vars[i].get().strip()
                        
                        if 1 <= channel_num <= 8 and channel_id:  # Validate channel number and ID
                            channels.append(channel_num)
                            channel_names[channel_num] = channel_id
                        else:
                            raise ValueError(f"Invalid channel configuration at row {i+1}")
                    except ValueError as e:
                        messagebox.showerror("Error", f"Channel configuration error: {str(e)}")
                        return
            
            if not channels:
                messagebox.showerror("Error", "No channels enabled or configured")
                return
            
            interval = int(self.tc08_interval_var.get())
            
            # Update config with new channel mapping
            self.config.tc08_channels = channels
            self.config.channel_names = channel_names
            
            # Create configuration
            config = TC08Config(
                channels=channels,
                sample_interval_ms=interval,
                thermocouple_type='K'
            )
            
            self.tc08_controller = TC08Controller(config)
            
            if self.tc08_controller.connect():
                # Start periodic health logging for long experiments
                self.tc08_controller.start_periodic_health_logging(interval_minutes=30)
                
                # Update temperature data storage for new channels
                self.temperature_data = {ch: [] for ch in channels}
                self.temperature_timestamps = {ch: [] for ch in channels}
                
                # Update plot with new channel configuration
                self.update_plot_configuration()
                
                # Start temperature streaming and plotting immediately
                if self.tc08_controller.start_streaming():
                    self.plotting_active = True
                    # Give TC-08 a moment to start collecting data
                    time.sleep(1.0)
                
                self.tc08_status_canvas.itemconfig(self.tc08_status_circle, fill="green")
                self.tc08_connect_btn.config(text="Disconnect TC-08", command=self.disconnect_tc08)
                
                # Show connection summary
                channel_summary = ", ".join([f"Ch{ch}({channel_names[ch]})" for ch in channels])
                # messagebox.showinfo("Success", f"TC-08 connected successfully!\nChannels: {channel_summary}\nTemperature plotting started.")
                self.update_button_states()  # Update all button states
            else:
                self.tc08_status_canvas.itemconfig(self.tc08_status_circle, fill="red")
                messagebox.showerror("Error", "Failed to connect to TC-08")
                
        except Exception as e:
            messagebox.showerror("Error", f"TC-08 connection error: {str(e)}")
    
    def disconnect_tc08(self):
        """Disconnect from TC-08"""
        try:
            # Stop any automated control first
            if self.static_control_active:
                self.static_control_active = False
                self.control_status_var.set("Manual Mode - TC-08 Disconnected")
                self.static_control_btn.config(text="Start Static Control")
                if self.temp_control_controller:
                    self.temp_control_controller.emergency_stop()
            
            # Stop plotting when TC-08 is disconnected
            self.plotting_active = False
            
            # Safely disconnect TC-08
            if self.tc08_controller:
                try:
                    self.tc08_controller.stop_streaming()
                    self.tc08_controller.disconnect()
                except Exception:
                    pass  # Silently handle disconnection errors
                finally:
                    self.tc08_controller = None
            
            # Update UI
            self.tc08_status_canvas.itemconfig(self.tc08_status_circle, fill="red")
            self.tc08_connect_btn.config(text="Connect TC-08", command=self.connect_tc08)
            self.update_button_states()  # Update all button states
            
        except Exception:
            # Force cleanup even if there are errors
            self.tc08_controller = None
            self.plotting_active = False
            self.tc08_status_canvas.itemconfig(self.tc08_status_circle, fill="red")
            self.tc08_connect_btn.config(text="Connect TC-08", command=self.connect_tc08)
    
    def connect_ir_sensor(self):
        """Connect to IR sensor Arduino"""
        try:
            port = self.ir_com_var.get()
            if not port:
                messagebox.showerror("Error", "Please select a COM port")
                return
                
            self.ir_sensor_controller = IRSensorController(port)
            
            if self.ir_sensor_controller.connect("IR_MONITOR_READY"):
                self.ir_status_canvas.itemconfig(self.ir_status_circle, fill="green")
                self.ir_connect_btn.config(text="Disconnect", command=self.disconnect_ir_sensor)
                self._ir_warning_shown = False  # Reset warning flag
                self.update_monitoring_button_state()  # Check if all devices connected
            else:
                self.ir_status_canvas.itemconfig(self.ir_status_circle, fill="red")
                messagebox.showerror("Error", f"Wrong Arduino type on {port}.\nExpected IR Monitor (IR_MONITOR_READY)")
                self.ir_sensor_controller = None
                
        except Exception as e:
            messagebox.showerror("Error", f"IR sensor connection error: {str(e)}")
            self.ir_sensor_controller = None
    
    def disconnect_ir_sensor(self):
        """Disconnect from IR sensor Arduino"""
        if self.ir_sensor_controller:
            self.ir_sensor_controller.disconnect()
            self.ir_sensor_controller = None
        
        self.ir_status_canvas.itemconfig(self.ir_status_circle, fill="red")
        self.ir_connect_btn.config(text="Connect", command=self.connect_ir_sensor)
        self.update_monitoring_button_state()  # Update monitoring button state
    
    def connect_temp_control(self):
        """Connect to DAQ temperature control device"""
        try:
            device_name = self.daq_device_var.get().strip()
            if not device_name:
                messagebox.showerror("Error", "Please enter a DAQ device name (e.g., Dev1)")
                return
                
            self.temp_control_controller = DAQTempController(device_name)
            
            if self.temp_control_controller.connect():
                self.temp_status_canvas.itemconfig(self.temp_status_circle, fill="green")
                self.temp_connect_btn.config(text="Disconnect", command=self.disconnect_temp_control)
                self._temp_warning_shown = False  # Reset warning flag
                self.update_monitoring_button_state()  # Check if all devices connected
                messagebox.showinfo("Success", f"DAQ temperature controller connected: {device_name}")
            else:
                self.temp_status_canvas.itemconfig(self.temp_status_circle, fill="red")
                messagebox.showerror("Error", f"Failed to connect to DAQ device {device_name}. Check device name and connections.")
                self.temp_control_controller = None
                
        except Exception as e:
            messagebox.showerror("Error", f"DAQ connection error: {str(e)}")
            self.temp_control_controller = None
    
    def disconnect_temp_control(self):
        """Disconnect from DAQ temperature control device"""
        if self.temp_control_controller:
            # Emergency stop before disconnecting
            self.temp_control_controller.emergency_stop()
            self.temp_control_controller.disconnect()
            self.temp_control_controller = None
        
        self.temp_status_canvas.itemconfig(self.temp_status_circle, fill="red")
        self.temp_connect_btn.config(text="Connect", command=self.connect_temp_control)
        self.update_monitoring_button_state()  # Update monitoring button state
    
    def start_recording(self):
        """Start comprehensive data recording with file selection and metadata"""
        if not self.tc08_controller:
            messagebox.showerror("Error", "TC-08 not connected")
            return
        
        if not self.ir_sensor_controller:
            messagebox.showerror("Error", "IR sensor not connected")
            return
        
        if not self.temp_control_controller:
            messagebox.showerror("Error", "Temperature control not connected")
            return
        
        # Get Fish ID from user input
        fish_id = self.fish_id_var.get().strip()
        if not fish_id:
            messagebox.showerror("Error", "Please enter a Fish ID before starting recording")
            return
        
        # Ask user to select filename for CSV data recording
        csv_filename = filedialog.asksaveasfilename(
            title="Save Trial Data As",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=f"trial_{fish_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        
        if not csv_filename:
            return  # User cancelled
        
        # Generate metadata filename
        metadata_filename = csv_filename.replace('.csv', '_metadata.txt')
        
        # Create metadata file
        try:
            with open(metadata_filename, 'w') as f:
                f.write(f"Trial Metadata\n")
                f.write(f"================\n\n")
                f.write(f"Fish ID: {fish_id}\n")
                f.write(f"Start Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Data File: {csv_filename}\n\n")
                
                # System Configuration
                f.write(f"System Configuration:\n")
                f.write(f"--------------------\n")
                f.write(f"TC-08 Channels: {self.config.tc08_channels}\n")
                
                # Compute warm and cold channels dynamically (same logic as used elsewhere)
                warm_channels = [ch for i, ch in enumerate(self.config.tc08_channels) if i < 3]
                cold_channels = [ch for i, ch in enumerate(self.config.tc08_channels) if i >= 3]
                f.write(f"Warm Side Channels: {warm_channels}\n")
                f.write(f"Cold Side Channels: {cold_channels}\n")
                
                # Use actual connected devices instead of config defaults
                ir_port = self.ir_com_var.get() if self.ir_sensor_controller else "Not connected"
                daq_device = self.daq_device_var.get() if self.temp_control_controller else "Not connected"
                f.write(f"IR Sensor Port: {ir_port}\n")
                f.write(f"DAQ Device: {daq_device}\n\n")
                
                # Control Settings
                f.write(f"Control Settings:\n")
                f.write(f"----------------\n")
                f.write(f"Start Temperature: {self.start_temp_var.get()}°C\n")
                f.write(f"Hysteresis: {self.hysteresis_var.get()}°C\n")
                f.write(f"Temperature Difference: {self.temp_diff_var.get()}°C\n")
                f.write(f"Start Side: {self.start_side_var.get()}\n\n")
                
                # Initial State
                f.write(f"Initial State:\n")
                f.write(f"-------------\n")
                f.write(f"Control Mode: {self.control_status_var.get()}\n")
                f.write(f"Fish Position: {self.fish_position_var.get()}\n")
                
        except Exception as e:
            messagebox.showerror("Error", f"Failed to create metadata file: {e}")
            return
        
        # Ensure TC-08 is streaming (should already be if connected)
        if not self.plotting_active:
            if not self.tc08_controller.start_streaming():
                messagebox.showerror("Error", "Failed to start temperature streaming")
                return
            else:
                # Successfully started streaming, now plotting should be active
                self.plotting_active = True
        
        # Initialize recording data storage
        self.recording_data = []
        self.recording_filename = csv_filename
        self.recording_metadata_filename = metadata_filename
        self.last_flush_time = time.time()
        
        # Create CSV header
        header = [
            "Timestamp",
            "Fish_Position",
            "Control_Mode",
            "Fish_Temperature",
            "Warm_Side_Avg",
            "Cold_Side_Avg"
        ]
        
        # Add individual channel headers
        for channel in self.config.tc08_channels:
            header.append(f"TC08_Ch{channel}")
        
        # Add control state headers
        header.extend(["Heating_Active", "Cooling_Active"])
        
        # Write header to CSV file
        try:
            with open(csv_filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to create CSV file: {e}")
            return
        
        self.recording_active = True
        self.monitoring_active = True  # For backward compatibility
        self.start_recording_btn.config(state=tk.DISABLED)
        self.stop_recording_btn.config(state=tk.NORMAL)
        
        # Disable all connection controls during recording
        self._set_connection_controls_state(tk.DISABLED)
        
        # Clear existing data for fresh recording
        for channel in self.config.tc08_channels:
            self.temperature_data[channel].clear()
            self.temperature_timestamps[channel].clear()
        
        # Clear average and fish temperature data for fresh recording
        self.warm_avg_data.clear()
        self.cold_avg_data.clear()
        self.avg_timestamps.clear()
        self.fish_temp_data.clear()
        
        messagebox.showinfo("Recording Started", 
                           f"Trial recording started for Fish ID: {fish_id}\n"
                           f"Data file: {csv_filename}\n"
                           f"Metadata file: {metadata_filename}")
    
    def record_data_point(self):
        """Record a single data point to CSV file and memory"""
        if not self.recording_active:
            return
        
        try:
            # Get current timestamp
            current_time = datetime.now()
            timestamp_str = current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # Include milliseconds
            
            # Collect data from all sources
            fish_position_raw = self.fish_position_var.get()
            control_mode_raw = self.control_status_var.get()
            
            # Clean up fish position - extract just the position name
            if "Position: " in fish_position_raw:
                fish_position = fish_position_raw.replace("Position: ", "")
            else:
                fish_position = fish_position_raw
            
            # Clean up control mode - extract just the mode type
            if "Static Control:" in control_mode_raw:
                control_mode = "Static Control"
            elif "Dynamic Control:" in control_mode_raw:
                control_mode = "Dynamic Control"
            elif "Manual Mode" in control_mode_raw:
                control_mode = "Manual Mode"
            else:
                # For any other status, try to extract the first part before ":"
                if ":" in control_mode_raw:
                    control_mode = control_mode_raw.split(":")[0]
                else:
                    control_mode = control_mode_raw
            
            # Get temperature data
            fish_temp = None
            warm_avg = None
            cold_avg = None
            channel_temps = {}
            
            if self.fish_temp_data and len(self.fish_temp_data) > 0:
                fish_temp = self.fish_temp_data[-1]
            
            if self.warm_avg_data and len(self.warm_avg_data) > 0:
                warm_avg = self.warm_avg_data[-1]
                
            if self.cold_avg_data and len(self.cold_avg_data) > 0:
                cold_avg = self.cold_avg_data[-1]
            
            # Get individual channel temperatures
            for channel in self.config.tc08_channels:
                if channel in self.temperature_data and len(self.temperature_data[channel]) > 0:
                    channel_temps[channel] = self.temperature_data[channel][-1]
                else:
                    channel_temps[channel] = None
            
            # Get control states
            heating_active = False
            cooling_active = False
            if self.temp_control_controller:
                try:
                    status = self.temp_control_controller.get_relay_sync_status()
                    # Extract heating and cooling status from sync status
                    heating_active = status.get('HEAT', {}).get('actual', False)
                    cooling_active = status.get('COOL', {}).get('actual', False)
                except Exception as e:
                    heating_active = False
                    cooling_active = False
            
            # Create data row
            data_row = [
                timestamp_str,
                fish_position,
                control_mode,
                fish_temp,
                warm_avg,
                cold_avg
            ]
            
            # Add individual channel temperatures
            for channel in self.config.tc08_channels:
                data_row.append(channel_temps.get(channel))
            
            # Add control states
            data_row.extend([heating_active, cooling_active])
            
            # Store in memory
            self.recording_data.append(data_row)
            
            # Check if it's time to flush to file
            current_time_sec = time.time()
            if current_time_sec - self.last_flush_time >= self.flush_interval:
                self.flush_recording_data()
                self.last_flush_time = current_time_sec
                
        except Exception as e:
            print(f"Error recording data point: {e}")
            import traceback
            traceback.print_exc()
    
    def flush_recording_data(self):
        """Flush accumulated recording data to CSV file"""
        if not self.recording_data or not self.recording_filename:
            return
        
        try:
            with open(self.recording_filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(self.recording_data)
            
            # Clear the buffer after successful write
            data_points_written = len(self.recording_data)
            self.recording_data.clear()
            
        except Exception as e:
            print(f"Error flushing recording data: {e}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Recording Error", f"Failed to save data: {e}")
    
    
    def stop_recording(self):
        """Stop comprehensive data recording"""
        # Flush any remaining data to file
        if self.recording_active and hasattr(self, 'recording_data'):
            self.flush_recording_data()
        
        # Update metadata with end time
        if hasattr(self, 'recording_metadata_filename') and self.recording_metadata_filename:
            try:
                with open(self.recording_metadata_filename, 'a') as f:
                    f.write(f"\nEnd Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    f.write(f"Recording completed successfully.\n")
            except Exception as e:
                print(f"Error updating metadata: {e}")
        
        self.recording_active = False
        self.monitoring_active = False  # For backward compatibility
        self.stop_recording_btn.config(state=tk.DISABLED)
        
        # Re-enable all connection controls after recording stops
        self._set_connection_controls_state(tk.NORMAL)
        
        # Update button states
        self.update_button_states()
        
        # Show completion message
        if hasattr(self, 'recording_filename') and self.recording_filename:
            messagebox.showinfo("Recording Stopped", 
                               f"Trial recording completed successfully.\n"
                               f"Data saved to: {self.recording_filename}")
    
    def manual_temp_control(self, control_type: str, enable: bool):
        """Manual temperature control"""
        if not self.temp_control_controller:
            messagebox.showwarning("Warning", "Temperature control not connected")
            return
        
        if control_type == 'heat':
            self.temp_control_controller.set_heating(enable)
        elif control_type == 'cool':
            self.temp_control_controller.set_cooling(enable)
        
        # Update relay status indicators immediately
        self.update_relay_status_indicators()
    
    def emergency_stop(self):
        """Emergency stop all temperature control"""
        if self.temp_control_controller:
            self.temp_control_controller.emergency_stop()
        
        # Update relay status indicators immediately
        self.update_relay_status_indicators()
        
        # Stop automated control modes
        self.static_control_active = False
        self.dynamic_control_active = False
        self.difference_control_active = False
        self.control_status_var.set("Emergency Stop - Manual Mode")
        self.static_control_btn.config(text="Start Static Control")
        self.dynamic_control_btn.config(text="Start Dynamic Control")
        self.difference_control_btn.config(text="Difference Control")
        
        # Update button states
        self.update_button_states()
        
        messagebox.showinfo("Emergency Stop", "All temperature control systems stopped!")
    
    def toggle_static_control(self):
        """Toggle static temperature control mode"""
        if not self.tc08_controller or not self.plotting_active:
            messagebox.showwarning("Warning", "TC-08 must be connected and plotting")
            return
            
        if not self.temp_control_controller:
            messagebox.showwarning("Warning", "Temperature control not connected")
            return
        
        if self.static_control_active:
            # Stop static control
            self.static_control_active = False
            self.control_status_var.set("Manual Mode")
            self.static_control_btn.config(text="Start Static Control")
            # Turn off all controls
            self.temp_control_controller.emergency_stop()
            # Update button states
            self.update_button_states()
        else:
            # Start static control
            try:
                # Validate parameters
                start_temp = float(self.start_temp_var.get())
                hysteresis = float(self.hysteresis_var.get())
                temp_diff = float(self.temp_diff_var.get())
                start_side = self.start_side_var.get()
                
                # Enhanced parameter validation
                if hysteresis <= 0:
                    raise ValueError("Hysteresis must be positive (recommended: 0.1-1.0°C)")
                if temp_diff <= 0:
                    raise ValueError("Temperature difference must be positive")
                if hysteresis > 5.0:
                    raise ValueError("Hysteresis too large (recommended: < 5.0°C)")
                if temp_diff > 20.0:
                    raise ValueError("Temperature difference too large (recommended: < 20.0°C)")
                if start_temp < 5.0 or start_temp > 40.0:
                    raise ValueError("Start temperature out of reasonable range (5-40°C)")
                if start_side not in ["warm", "cold"]:
                    raise ValueError("Start side must be 'warm' or 'cold'")
                
            # Check if we have valid temperature readings before starting
                # Try multiple times to get readings as TC-08 might need a moment
                warm_avg = None
                cold_avg = None
                
                for attempt in range(3):
                    warm_avg = self.get_warm_side_average()
                    cold_avg = self.get_cold_side_average()
                    if warm_avg is not None and cold_avg is not None:
                        break
                    time.sleep(0.5)  # Wait 500ms between attempts
                
                if warm_avg is None or cold_avg is None:
                    raise ValueError("Cannot start control - no valid temperature readings available")
                
                # Stop dynamic control if active
                if self.dynamic_control_active:
                    self.dynamic_control_active = False
                    self.difference_control_active = False  # Also stop difference control
                    self.dynamic_control_btn.config(text="Start Dynamic Control")
                    self.difference_control_btn.config(text="Difference Control")
                
                self.static_control_active = True
                self.control_status_var.set(f"Static Control: {start_side.title()} Side Start, {start_temp}°C ±{hysteresis}°C")
                self.static_control_btn.config(text="Stop Static Control")
                
                # Update button states
                self.update_button_states()
                
            except ValueError as e:
                messagebox.showerror("Error", f"Invalid control parameters: {e}")
    
    def toggle_dynamic_control(self):
        """Toggle dynamic temperature control mode"""
        if not self.tc08_controller or not self.plotting_active:
            messagebox.showwarning("Warning", "TC-08 must be connected and plotting")
            return
            
        if not self.temp_control_controller:
            messagebox.showwarning("Warning", "Temperature control not connected")
            return
        
        if not self.ir_sensor_controller and not self.persistent_manual_mode:
            messagebox.showwarning("Warning", "IR sensor must be connected for dynamic control\n(unless using persistent manual mode)")
            return
        
        if self.dynamic_control_active:
            # Stop dynamic control
            self.dynamic_control_active = False
            self.difference_control_active = False  # Also stop difference control
            self.control_status_var.set("Manual Mode")
            self.dynamic_control_btn.config(text="Start Dynamic Control")
            self.difference_control_btn.config(text="Difference Control")
            # Turn off all controls
            self.temp_control_controller.emergency_stop()
            # Update button states
            self.update_button_states()
        else:
            # Start dynamic control
            try:
                # Validate parameters
                temp_diff = float(self.temp_diff_var.get())
                
                # Enhanced parameter validation
                if temp_diff <= 0:
                    raise ValueError("Temperature difference must be positive")
                if temp_diff > 20.0:
                    raise ValueError("Temperature difference too large (recommended: < 20.0°C)")
                
                # Check if we have valid temperature readings and fish position
                warm_avg = self.get_warm_side_average()
                cold_avg = self.get_cold_side_average()
                
                if warm_avg is None or cold_avg is None:
                    raise ValueError("Cannot start control - no valid temperature readings available")
                
                # Check if we have valid fish position (IR sensor OR persistent manual mode)
                current_position = self.get_current_fish_position()
                if current_position is None or current_position not in [0, 1, 2]:
                    if self.persistent_manual_mode:
                        raise ValueError("Cannot start control - no manual position set in persistent mode")
                    else:
                        raise ValueError("Cannot start control - invalid fish position data from IR sensor")
                
                # Stop static control if active
                if self.static_control_active:
                    self.static_control_active = False
                    self.static_control_btn.config(text="Start Static Control")
                
                self.dynamic_control_active = True
                self.control_status_var.set(f"Dynamic Control: Following Fish Position, Target Diff={temp_diff}°C")
                self.dynamic_control_btn.config(text="Stop Dynamic Control")
                
                # Update button states (enable difference control button)
                self.update_button_states()
                
            except ValueError as e:
                messagebox.showerror("Error", f"Invalid control parameters: {e}")
    
    def toggle_difference_control(self):
        """Toggle difference control mode (only available during dynamic control)"""
        if not self.dynamic_control_active:
            messagebox.showwarning("Warning", "Difference control only available during dynamic control")
            return
        
        if self.difference_control_active:
            # Stop difference control
            self.difference_control_active = False
            self.difference_control_btn.config(text="Difference Control")
        else:
            # Start difference control
            self.difference_control_active = True
            self.difference_control_btn.config(text="Stop Diff Control")
            
        # Update status display
        temp_diff = float(self.temp_diff_var.get())
        diff_status = " + Diff Control" if self.difference_control_active else ""
        self.control_status_var.set(f"Dynamic Control: Following Fish Position, Target Diff={temp_diff}°C{diff_status}")
    
    def get_warm_side_average(self):
        """Calculate average temperature of warm side channels using latest stored data"""
        try:
            if not self.tc08_controller or not self.plotting_active:
                return None
            
            # Use the same logic as plotting - get most recent stored data
            warm_channels = []
            for i, channel in enumerate(self.config.tc08_channels):
                # Use first half of channels as warm side
                if i < len(self.config.tc08_channels) // 2:
                    warm_channels.append(channel)
            
            valid_temps = []
            for channel in warm_channels:
                # Use stored temperature data (same as plotting)
                if (channel in self.temperature_data and 
                    len(self.temperature_data[channel]) > 0):
                    latest_temp = self.temperature_data[channel][-1]  # Most recent value
                    valid_temps.append(latest_temp)
            
            if len(valid_temps) == 0:
                return None
                
            avg = sum(valid_temps) / len(valid_temps)
            return avg
        except Exception as e:
            return None
    
    def get_cold_side_average(self):
        """Calculate average temperature of cold side channels using latest stored data"""
        try:
            if not self.tc08_controller or not self.plotting_active:
                return None
            
            # Use the same logic as plotting - get most recent stored data
            cold_channels = []
            for i, channel in enumerate(self.config.tc08_channels):
                # Use second half of channels as cold side
                if i >= len(self.config.tc08_channels) // 2:
                    cold_channels.append(channel)
            
            valid_temps = []
            for channel in cold_channels:
                # Use stored temperature data (same as plotting)
                if (channel in self.temperature_data and 
                    len(self.temperature_data[channel]) > 0):
                    latest_temp = self.temperature_data[channel][-1]  # Most recent value
                    valid_temps.append(latest_temp)
            
            if len(valid_temps) == 0:
                return None
                
            avg = sum(valid_temps) / len(valid_temps)
            return avg
        except Exception as e:
            return None
    
    def get_fish_temperature(self):
        """Calculate fish temperature based on current position"""
        try:
            # Only calculate if both TC-08 and IR sensor are connected
            if not self.tc08_controller or not self.plotting_active or not self.ir_sensor_controller:
                return None
            
            # Get current fish position (with manual override support)
            fish_position = self.get_current_fish_position()
            
            # Get warm and cold averages
            warm_avg = self.get_warm_side_average()
            cold_avg = self.get_cold_side_average()
            
            if warm_avg is None or cold_avg is None:
                return None
            
            # Calculate fish temperature based on position
            if fish_position == 2:  # Right side (warm)
                return warm_avg
            elif fish_position == 0:  # Passage (average of both sides)
                return (warm_avg + cold_avg) / 2
            elif fish_position == 1:  # Left side (cold)
                return cold_avg
            else:
                return None  # Unknown position
                
        except Exception as e:
            return None
    
    def set_manual_fish_position(self, position):
        """Manually override fish position (for sensor glitches or testing)"""
        try:
            position_names = {0: "Passage", 1: "Left (Cool)", 2: "Right (Warm)"}
            position_name = position_names.get(position, "Unknown")
            
            if self.persistent_manual_mode:
                # Persistent manual mode - direct setting without confirmation
                self.manual_position_override = position
                self.fish_position_var.set(f"Position: {position_name} (MANUAL - PERSISTENT)")
                self.update_position_buttons(position)
            else:
                # Momentary correction mode - confirm and set temporary override
                result = messagebox.askyesno("Momentary Position Correction", 
                                           f"Correct fish position to: {position_name}\n\n"
                                           "This will temporarily override the IR sensor reading\n"
                                           "to correct momentary glitches.\n\n"
                                           "Control will return to IR sensor after correction.")
                
                if result:
                    self.manual_position_override = position
                    
                    # Override the IR sensor's stored position so it persists when returning to automatic
                    if self.ir_sensor_controller:
                        self.ir_sensor_controller.override_fish_position(position)
                    
                    # Update position display immediately
                    self.fish_position_var.set(f"Position: {position_name} (CORRECTING)")
                    self.update_position_buttons(position)
                    
                    # Schedule return to automatic mode after brief delay
                    self.root.after(3000, self.return_to_automatic_after_correction)  # 3 seconds
                    
        except Exception as e:
            messagebox.showerror("Error", f"Failed to set manual position: {e}")
    
    def return_to_automatic_after_correction(self):
        """Return to automatic IR sensor control after momentary correction"""
        if not self.persistent_manual_mode and self.manual_position_override is not None:
            self.manual_position_override = None
            
            # Update display to show automatic mode
            if self.ir_sensor_controller:
                try:
                    ir_data = self.ir_sensor_controller.get_latest_data()
                    position_names = {0: "Passage", 1: "Left (Cool)", 2: "Right (Warm)"}
                    position_text = f"Position: {position_names.get(ir_data['fish_position'], 'Unknown')}"
                    self.fish_position_var.set(position_text)
                    self.update_position_buttons(ir_data['fish_position'])
                except:
                    self.fish_position_var.set("Position: Unknown")
                    self.update_position_buttons(None)
            else:
                self.fish_position_var.set("Position: Unknown")
                self.update_position_buttons(None)
    
    def clear_manual_position_override(self):
        """Clear manual position override and return to automatic detection"""
        self.manual_position_override = None
        
        # Hide the clear override button
        if hasattr(self, 'clear_override_btn'):
            self.clear_override_btn.destroy()
            delattr(self, 'clear_override_btn')
        
        # Update display to show automatic mode
        if self.ir_sensor_controller:
            ir_data = self.ir_sensor_controller.get_latest_data()
            position_names = {0: "Passage", 1: "Left (Cool)", 2: "Right (Warm)"}
            position_text = f"Position: {position_names.get(ir_data['fish_position'], 'Unknown')}"
            self.fish_position_var.set(position_text)
            self.update_position_buttons(ir_data['fish_position'])
        else:
            self.fish_position_var.set("Position: Unknown")
            self.update_position_buttons(None)
        
        messagebox.showinfo("Override Cleared", "Returned to automatic fish position detection.")
    
    def create_clear_override_button(self):
        """Create the clear override button"""
        if not hasattr(self, 'clear_override_btn'):
            # Find the fish frame to add the button
            for widget in self.root.winfo_children():
                self._find_fish_frame_and_add_button(widget)
    
    def _find_fish_frame_and_add_button(self, widget):
        """Recursively find the fish frame and add clear override button"""
        try:
            if (hasattr(widget, 'winfo_class') and 
                hasattr(widget, 'cget') and
                widget.cget('text') == 'Fish Position'):
                
                # Create clear override button
                self.clear_override_btn = ttk.Button(widget, text="Clear Override", 
                                                   command=self.clear_manual_position_override)
                self.clear_override_btn.pack(pady=5)
                return True
        except:
            pass
        
        # Recurse through children
        for child in widget.winfo_children():
            if self._find_fish_frame_and_add_button(child):
                return True
        return False
    
    def toggle_persistent_manual_control(self):
        """Toggle persistent manual control mode"""
        self.persistent_manual_mode = self.persistent_manual_var.get()
        
        if self.persistent_manual_mode:
            # Entering persistent manual mode
            messagebox.showinfo("Persistent Manual Control", 
                              "Persistent manual control enabled.\n\n"
                              "• Click position buttons to set fish location\n"
                              "• IR sensor not required in this mode\n"
                              "• Position will remain set until changed manually\n"
                              "• Uncheck to return to IR sensor control")
            
            # If no position is set, default to passage
            if self.manual_position_override is None:
                self.manual_position_override = 0  # Default to passage
                self.fish_position_var.set("Position: Passage (MANUAL - PERSISTENT)")
                self.update_position_buttons(0)
        else:
            # Exiting persistent manual mode
            self.manual_position_override = None
            
            # Update display based on IR sensor if available
            if self.ir_sensor_controller:
                try:
                    ir_data = self.ir_sensor_controller.get_latest_data()
                    position_names = {0: "Passage", 1: "Left (Cool)", 2: "Right (Warm)"}
                    position_text = f"Position: {position_names.get(ir_data['fish_position'], 'Unknown')}"
                    self.fish_position_var.set(position_text)
                    self.update_position_buttons(ir_data['fish_position'])
                except:
                    self.fish_position_var.set("Position: Unknown")
                    self.update_position_buttons(None)
            else:
                self.fish_position_var.set("Position: Unknown")
                self.update_position_buttons(None)
    
    def get_current_fish_position(self):
        """Get current fish position (manual override takes precedence over sensor)"""
        if self.manual_position_override is not None:
            return self.manual_position_override
        elif self.ir_sensor_controller:
            ir_data = self.ir_sensor_controller.get_latest_data()
            return ir_data['fish_position']
        else:
            return None
    
    def update_position_buttons(self, current_position):
        """Update position button colors to show current fish location"""
        for arduino_pos, button in self.position_buttons.items():
            if arduino_pos == current_position:
                # Highlight current position with light green
                button.config(bg='lightgreen')
            else:
                # Normal gray background
                button.config(bg='lightgray')
    
    def perform_static_temperature_control(self):
        """Perform static temperature control logic"""
        try:
            # Get control parameters
            start_temp = float(self.start_temp_var.get())
            hysteresis = float(self.hysteresis_var.get())
            temp_diff = float(self.temp_diff_var.get())
            start_side = self.start_side_var.get()
            
            # Get current temperatures
            warm_avg = self.get_warm_side_average()
            cold_avg = self.get_cold_side_average()
            
            if warm_avg is None or cold_avg is None:
                return  # Can't control without valid readings
            
            # Calculate target temperatures based on start side
            if start_side == "warm":
                warm_target = start_temp
                cold_target = start_temp - temp_diff
            else:  # start_side == "cold"
                cold_target = start_temp
                warm_target = start_temp + temp_diff
            
            # Control warm side
            if warm_avg < warm_target - hysteresis:
                # Too cold on warm side - turn on heating
                self.temp_control_controller.set_heating(True)
                # self.temp_control_controller.set_cooling(False)
            elif warm_avg > warm_target:
                # Too warm on warm side - turn off heating
                self.temp_control_controller.set_heating(False)
            
            # Control cold side
            if cold_avg > cold_target + hysteresis:
                # Too warm on cold side - turn on cooling
                self.temp_control_controller.set_cooling(True)
                # self.temp_control_controller.set_heating(False)
            elif cold_avg < cold_target:
                # Too cold on cold side - turn off cooling
                self.temp_control_controller.set_cooling(False)
            
            # Check temperature difference between sides
            actual_diff = abs(warm_avg - cold_avg)
            if actual_diff > temp_diff + hysteresis:
                # Temperature difference too large - activate buffer tanks
                self.temp_control_controller.set_buffer_heating(True)
                self.temp_control_controller.set_buffer_cooling(True)
            elif actual_diff < temp_diff:
                # Temperature difference acceptable - turn off buffer tanks
                self.temp_control_controller.set_buffer_heating(False)
                self.temp_control_controller.set_buffer_cooling(False)
            
            # Update status with current readings (relay indicators are now visual)
            actual_diff = abs(warm_avg - cold_avg)
            status_text = (f"Static Control: W={warm_avg:.1f}°C (target {warm_target:.1f}°C), "
                          f"C={cold_avg:.1f}°C (target {cold_target:.1f}°C), "
                          f"Diff={actual_diff:.1f}°C (target {temp_diff:.1f}°C)")
            self.control_status_var.set(status_text)
            
        except Exception as e:
            # On error, stop automated control for safety
            self.static_control_active = False
            self.control_status_var.set("Control Error - Manual Mode")
            self.static_control_btn.config(text="Start Static Control")
    
    def perform_dynamic_temperature_control(self):
        """Perform dynamic temperature control based on fish position"""
        try:
            # Get control parameters
            temp_diff = float(self.temp_diff_var.get())
            hysteresis = float(self.hysteresis_var.get())

            # Get current temperatures
            warm_avg = self.get_warm_side_average()
            cold_avg = self.get_cold_side_average()
            
            if warm_avg is None or cold_avg is None:
                return  # Can't control without valid readings
            
            # Get current fish position (with manual override support)
            fish_position = self.get_current_fish_position()
            
            # Calculate actual temperature difference
            actual_diff = abs(warm_avg - cold_avg)
            
            # Control logic: prioritize difference control when active
            if self.difference_control_active:
                # Active difference control mode
                if actual_diff < temp_diff:
                    # Temperature difference too small - activate opposing control to increase difference
                    # Get current relay states first to avoid conflicts
                    relay_states = self.temp_control_controller.get_status()
                    current_heating = relay_states.get("HEAT", False)
                    current_cooling = relay_states.get("COOL", False)
                    
                    # Fish position determines primary control, then add opposing control
                    if fish_position == 2:  # Right side (warm) - primary heating
                        self.temp_control_controller.set_heating(True)
                        if not current_cooling:  # Add cooling to increase difference
                            self.temp_control_controller.set_cooling(True)
                    elif fish_position == 1:  # Left side (cold) - primary cooling
                        self.temp_control_controller.set_cooling(True)
                        if not current_heating:  # Add heating to increase difference
                            self.temp_control_controller.set_heating(True)
                    elif fish_position == 0:  # Passage - maintain current state but ensure difference
                        if current_heating and not current_cooling:
                            self.temp_control_controller.set_cooling(True)
                        elif current_cooling and not current_heating:
                            self.temp_control_controller.set_heating(True)
                        # If both or neither are active, maintain current state
                
                elif actual_diff > temp_diff + hysteresis:
                    # Temperature difference too large - use standard dynamic control + buffer tanks
                    if fish_position == 2:  # Right side (warm) - keep heating, stop cooling
                        self.temp_control_controller.set_heating(True)
                        self.temp_control_controller.set_cooling(False)
                    elif fish_position == 1:  # Left side (cold) - keep cooling, stop heating
                        self.temp_control_controller.set_cooling(True)
                        self.temp_control_controller.set_heating(False)
                    # Activate buffer tanks for extreme differences
                    self.temp_control_controller.set_buffer_heating(True)
                    self.temp_control_controller.set_buffer_cooling(True)
                else:
                    # Difference is in acceptable range - use standard fish position control
                    if fish_position == 2:  # Right side (warm)
                        self.temp_control_controller.set_heating(True)
                        self.temp_control_controller.set_cooling(False)
                    elif fish_position == 1:  # Left side (cold)
                        self.temp_control_controller.set_cooling(True)
                        self.temp_control_controller.set_heating(False)
                    # Turn off buffer tanks when difference is acceptable
                    self.temp_control_controller.set_buffer_heating(False)
                    self.temp_control_controller.set_buffer_cooling(False)
                    
            else:
                # Standard dynamic control - fish position based control only
                # Only change heating/cooling when fish is on a side (completes side change)
                # Maintain current state when fish is in passage
                if fish_position == 2:  # Right side (warm)
                    # Fish is on warm side - activate heating, deactivate cooling
                    self.temp_control_controller.set_heating(True)
                    self.temp_control_controller.set_cooling(False)
                elif fish_position == 1:  # Left side (cold)
                    # Fish is on cold side - activate cooling, deactivate heating
                    self.temp_control_controller.set_cooling(True)
                    self.temp_control_controller.set_heating(False)
                # elif fish_position == 0:  # Passage
                #     # Fish in passage - maintain current heating/cooling state (do nothing)
                
                # Standard dynamic control - buffer tanks for extreme differences only
                if actual_diff > temp_diff + hysteresis:
                    # Temperature difference too large - activate buffer tanks
                    self.temp_control_controller.set_buffer_heating(True)
                    self.temp_control_controller.set_buffer_cooling(True)
                elif actual_diff < temp_diff:
                    # Temperature difference acceptable - turn off buffer tanks
                    self.temp_control_controller.set_buffer_heating(False)
                    self.temp_control_controller.set_buffer_cooling(False)
            
            # Update status with current readings and fish position
            position_names = {0: "Passage", 1: "Left (Cool)", 2: "Right (Warm)"}
            position_name = position_names.get(fish_position, "Unknown")
            diff_status = " + Diff Control" if self.difference_control_active else ""
            status_text = (f"Dynamic Control: Fish at {position_name}, "
                          f"W={warm_avg:.1f}°C, C={cold_avg:.1f}°C, "
                          f"Diff={actual_diff:.1f}°C (target {temp_diff:.1f}°C){diff_status}")
            self.control_status_var.set(status_text)
            
        except Exception as e:
            # On error, stop automated control for safety
            self.dynamic_control_active = False
            self.control_status_var.set("Control Error - Manual Mode")
            self.dynamic_control_btn.config(text="Start Dynamic Control")
    
    def update_gui(self):
        """Update GUI with latest data (called periodically)"""
        try:
            # Update temperature data and plot (when TC-08 connected and plotting)
            if self.plotting_active and self.tc08_controller:
                try:
                    self.update_temperature_plot()
                except RuntimeError as e:
                    # TC-08 communication failure
                    print(f"TC-08 communication error: {e}")
                    self.tc08_status_canvas.itemconfig(self.tc08_status_circle, fill="red")
                    self.plotting_active = False
                except Exception as e:
                    print(f"Unexpected error updating temperature plot: {e}")
            
            # Record data point if recording is active
            if self.recording_active:
                self.record_data_point()
            
            # Perform automated temperature control if active (requires plotting + temp controller)
            if self.static_control_active and self.plotting_active:
                self.perform_static_temperature_control()
            elif self.dynamic_control_active and self.plotting_active:
                self.perform_dynamic_temperature_control()
            
            # Update temperature control relay states and handle retries
            if self.temp_control_controller:
                self.update_temp_control_status()
            
            # Update IR sensor and fish position (whenever IR sensor is connected OR in persistent manual mode)
            if self.ir_sensor_controller or self.persistent_manual_mode:
                self.update_ir_visualization()
            
            # Check Arduino connection health
            self.check_arduino_connections()
            
        except Exception as e:
            pass  # Silently handle GUI update errors
        
        # Schedule next update
        self.update_timer = self.root.after(1000, self.update_gui)
    
    def check_arduino_connections(self):
        """Monitor Arduino connection health using heartbeat messages"""
        # Check IR sensor connection
        if self.ir_sensor_controller:
            if not self.ir_sensor_controller.is_connection_healthy():
                if not hasattr(self, '_ir_warning_shown') or not self._ir_warning_shown:
                    self.ir_status_canvas.itemconfig(self.ir_status_circle, fill="orange")
                    self._ir_warning_shown = True
                    if self.monitoring_active or self.recording_active:
                        messagebox.showerror("Connection Lost", 
                                           "IR Sensor Arduino connection lost!\nStopping operations for safety.")
                        if self.recording_active:
                            self.stop_recording()
                        if self.static_control_active:
                            self.static_control_active = False
                            self.control_status_var.set("Manual Mode - Connection Lost")
                            self.static_control_btn.config(text="Start Static Control")
            else:
                self._ir_warning_shown = False
                if self.ir_sensor_controller.connected:
                    self.ir_status_canvas.itemconfig(self.ir_status_circle, fill="green")
        
        # Check DAQ connection (no heartbeat, just check if tasks are valid)
        if self.temp_control_controller:
            if not self.temp_control_controller.is_connection_healthy():
                if not hasattr(self, '_temp_warning_shown') or not self._temp_warning_shown:
                    self.temp_status_canvas.itemconfig(self.temp_status_circle, fill="orange")
                    self._temp_warning_shown = True
                    if self.monitoring_active or self.recording_active:
                        messagebox.showerror("Connection Lost",
                                           "DAQ temperature controller connection lost!\nStopping operations for safety.")
                        if self.recording_active:
                            self.stop_recording()
                        if self.static_control_active:
                            self.static_control_active = False
                            self.control_status_var.set("Manual Mode - Connection Lost")
                            self.static_control_btn.config(text="Start Static Control")
            else:
                self._temp_warning_shown = False
                if self.temp_control_controller.connected:
                    self.temp_status_canvas.itemconfig(self.temp_status_circle, fill="green")
    
    def update_relay_status_indicators(self):
        """Update visual relay status indicators"""
        try:
            if not self.temp_control_controller:
                # Set all to gray when not connected
                self.heat_status_canvas.itemconfig(self.heat_status_circle, fill="gray")
                self.cool_status_canvas.itemconfig(self.cool_status_circle, fill="gray")
                self.bheat_status_canvas.itemconfig(self.bheat_status_circle, fill="gray")
                self.bcool_status_canvas.itemconfig(self.bcool_status_circle, fill="gray")
                return
            
            # Get current relay states
            relay_states = self.temp_control_controller.get_status()
            
            # Update heat relay indicator
            heat_color = "red" if relay_states.get("HEAT", False) else "lightgray"
            self.heat_status_canvas.itemconfig(self.heat_status_circle, fill=heat_color)
            
            # Update cool relay indicator
            cool_color = "blue" if relay_states.get("COOL", False) else "lightgray"
            self.cool_status_canvas.itemconfig(self.cool_status_circle, fill=cool_color)
            
            # Update buffer heat relay indicator
            bheat_color = "orange" if relay_states.get("BHEAT", False) else "lightgray"
            self.bheat_status_canvas.itemconfig(self.bheat_status_circle, fill=bheat_color)
            
            # Update buffer cool relay indicator
            bcool_color = "lightblue" if relay_states.get("BCOOL", False) else "lightgray"
            self.bcool_status_canvas.itemconfig(self.bcool_status_circle, fill=bcool_color)
            
        except Exception as e:
            # On error, set all to gray
            self.heat_status_canvas.itemconfig(self.heat_status_circle, fill="gray")
            self.cool_status_canvas.itemconfig(self.cool_status_circle, fill="gray")
            self.bheat_status_canvas.itemconfig(self.bheat_status_circle, fill="gray")
            self.bcool_status_canvas.itemconfig(self.bcool_status_circle, fill="gray")
    
    def update_temp_control_status(self):
        """Update temperature control relay states and handle failed commands"""
        try:
            # Get latest relay states from Arduino
            current_states = self.temp_control_controller.get_status()
            
            # Update visual relay indicators
            self.update_relay_status_indicators()
            
            # Send periodic ping to prevent Arduino timeout (every 15 seconds)
            if not hasattr(self, '_last_ping_time'):
                self._last_ping_time = 0
            
            current_time = time.time()
            if current_time - self._last_ping_time > 15:  # 15 second interval
                self.temp_control_controller.keep_alive()
                self._last_ping_time = current_time
            
            # Check relay synchronization and retry failed commands
            sync_status = self.temp_control_controller.get_relay_sync_status()
            
            # Retry any commands that timed out
            self.temp_control_controller.retry_failed_commands()
            
            # Optional: Update GUI with relay status indicators (can be implemented later)
            # This could show actual relay states vs desired states for debugging
            
        except Exception as e:
            pass  # Silently handle temperature control status errors
    
    def update_button_states(self):
        """Update all button states based on current system status"""
        
        # Recording button states (requires all three devices)
        all_connected = (
            self.tc08_controller is not None and 
            self.ir_sensor_controller is not None and 
            self.temp_control_controller is not None and
            self.tc08_controller.connected and
            self.ir_sensor_controller.connected and 
            self.temp_control_controller.connected
        )
        
        if all_connected and not self.recording_active:
            self.start_recording_btn.config(state=tk.NORMAL)
        else:
            self.start_recording_btn.config(state=tk.DISABLED)
        
        # Temperature control button states
        # Static control: requires TC-08 plotting + temp controller
        static_control_available = (
            self.tc08_controller is not None and 
            self.temp_control_controller is not None and
            self.tc08_controller.connected and
            self.temp_control_controller.connected and
            self.plotting_active
        )
        
        # Dynamic control: requires TC-08 plotting + temp controller + (IR sensor OR persistent manual mode)
        dynamic_control_available = (
            static_control_available and
            (self.persistent_manual_mode or 
             (self.ir_sensor_controller is not None and self.ir_sensor_controller.connected))
        )
        
        if static_control_available:
            self.static_control_btn.config(state=tk.NORMAL)
        else:
            self.static_control_btn.config(state=tk.DISABLED)
            
        if dynamic_control_available:
            self.dynamic_control_btn.config(state=tk.NORMAL)
        else:
            self.dynamic_control_btn.config(state=tk.DISABLED)
        
        # Difference control: only available when dynamic control is active
        if self.dynamic_control_active:
            self.difference_control_btn.config(state=tk.NORMAL)
        else:
            self.difference_control_btn.config(state=tk.DISABLED)
    
    def update_monitoring_button_state(self):
        """Legacy method - redirects to update_button_states for compatibility"""
        self.update_button_states()
    
    def _set_connection_controls_state(self, state):
        """Enable or disable all connection-related controls during monitoring"""
        # TC-08 channel configuration entries
        for i in range(6):
            if i in self.channel_vars:
                try:
                    # Find the entry widget for this channel number
                    for widget in self.root.winfo_children():
                        self._find_and_set_entry_state(widget, self.channel_vars[i], state)
                        self._find_and_set_entry_state(widget, self.channel_id_vars[i], state)
                        self._find_and_set_checkbutton_state(widget, self.channel_enable_vars[i], state)
                except:
                    pass
        
        # Sample interval and plot period entries
        try:
            for widget in self.root.winfo_children():
                self._find_and_set_entry_state(widget, self.tc08_interval_var, state)
                self._find_and_set_entry_state(widget, self.plot_period_var, state)
        except:
            pass
        
        # Temperature control parameter entries
        try:
            for widget in self.root.winfo_children():
                self._find_and_set_entry_state(widget, self.start_temp_var, state)
                self._find_and_set_entry_state(widget, self.hysteresis_var, state)
                self._find_and_set_entry_state(widget, self.temp_diff_var, state)
                self._find_and_set_combobox_state(widget, self.start_side_var, state)
        except:
            pass
        
        # COM port combo boxes and DAQ device combo box
        try:
            com_state = "readonly" if state == tk.DISABLED else "readonly"  # Keep readonly for dropdowns
            self.ir_com_combo.config(state=com_state if state == tk.NORMAL else tk.DISABLED)
            self.daq_device_combo.config(state=com_state if state == tk.NORMAL else tk.DISABLED)
        except:
            pass
        
        # Connection buttons - disable all during monitoring
        self.tc08_connect_btn.config(state=state)
        self.ir_connect_btn.config(state=state)
        self.temp_connect_btn.config(state=state)
    
    def _find_and_set_entry_state(self, widget, target_var, state):
        """Recursively find Entry widgets with specific textvariable and set their state"""
        try:
            if (hasattr(widget, 'winfo_class') and widget.winfo_class() == 'Entry' and
                hasattr(widget, 'cget') and widget.cget('textvariable') == str(target_var)):
                widget.config(state=state)
        except:
            pass
        
        # Recurse through children
        for child in widget.winfo_children():
            self._find_and_set_entry_state(child, target_var, state)
    
    def _find_and_set_checkbutton_state(self, widget, target_var, state):
        """Recursively find Checkbutton widgets with specific variable and set their state"""
        try:
            if (hasattr(widget, 'winfo_class') and widget.winfo_class() == 'Checkbutton' and
                hasattr(widget, 'cget') and widget.cget('variable') == str(target_var)):
                widget.config(state=state)
        except:
            pass
        
        # Recurse through children
        for child in widget.winfo_children():
            self._find_and_set_checkbutton_state(child, target_var, state)
    
    def _find_and_set_combobox_state(self, widget, target_var, state):
        """Recursively find Combobox widgets with specific textvariable and set their state"""
        try:
            if (hasattr(widget, 'winfo_class') and widget.winfo_class() == 'Combobox' and
                hasattr(widget, 'cget') and widget.cget('textvariable') == str(target_var)):
                combo_state = "readonly" if state == tk.NORMAL else tk.DISABLED
                widget.config(state=combo_state)
        except:
            pass
        
        # Recurse through children
        for child in widget.winfo_children():
            self._find_and_set_combobox_state(child, target_var, state)
    
    def update_temperature_plot(self):
        """Update temperature plot with new data and warm/cold averages"""
        try:
            # Check if TC-08 is still properly connected
            if not self.tc08_controller.connected:
                print("TC-08 connection lost - stopping temperature plotting")
                self.tc08_status_canvas.itemconfig(self.tc08_status_circle, fill="orange")
                return
            
            readings = self.tc08_controller.read_temperatures()
            
            # Check if all readings failed (sign of USB communication failure)
            valid_readings = sum(1 for r in readings.values() if not r.error)
            if valid_readings == 0 and len(readings) > 0:
                print("Warning: TC-08 returned no valid readings - possible USB communication degradation")
                self.tc08_status_canvas.itemconfig(self.tc08_status_circle, fill="orange")
                return
            current_time = datetime.now()
            
            # Get plot period from user input
            try:
                plot_period_seconds = float(self.plot_period_var.get())
            except:
                plot_period_seconds = 300  # Default 5 minutes
            
            # Calculate buffer size based on plot period and sample interval
            sample_interval_seconds = float(self.tc08_interval_var.get()) / 1000.0
            max_data_points = max(10, int(plot_period_seconds / sample_interval_seconds))
            
            # Add new data points
            for channel, reading in readings.items():
                if channel in self.temperature_data:  # Only update configured channels
                    if not reading.error and reading.temperature is not None:
                        self.temperature_data[channel].append(reading.temperature)
                        self.temperature_timestamps[channel].append(current_time)
                        
                        # Keep only data within plot period
                        cutoff_time = current_time - timedelta(seconds=plot_period_seconds)
                        
                        # Remove old data points beyond plot period
                        while (len(self.temperature_timestamps[channel]) > 0 and 
                               self.temperature_timestamps[channel][0] < cutoff_time):
                            self.temperature_data[channel].pop(0)
                            self.temperature_timestamps[channel].pop(0)
                        
                        # Also ensure we don't exceed max data points (safety limit)
                        if len(self.temperature_data[channel]) > max_data_points:
                            self.temperature_data[channel] = self.temperature_data[channel][-max_data_points:]
                            self.temperature_timestamps[channel] = self.temperature_timestamps[channel][-max_data_points:]
            
            # Calculate warm and cold averages
            warm_channels = [ch for i, ch in enumerate(self.config.tc08_channels) if i < 3]
            cold_channels = [ch for i, ch in enumerate(self.config.tc08_channels) if i >= 3]
            
            # Get current readings for averaging
            warm_temps = []
            cold_temps = []
            
            for channel, reading in readings.items():
                if not reading.error and reading.temperature is not None:
                    if channel in warm_channels:
                        warm_temps.append(reading.temperature)
                    elif channel in cold_channels:
                        cold_temps.append(reading.temperature)
            
            # Calculate and store averages if we have data
            if warm_temps:
                warm_avg = sum(warm_temps) / len(warm_temps)
                self.warm_avg_data.append(warm_avg)
                if len(self.avg_timestamps) == 0 or len(self.avg_timestamps) < len(self.warm_avg_data):
                    self.avg_timestamps.append(current_time)
                    
            if cold_temps:
                cold_avg = sum(cold_temps) / len(cold_temps)
                self.cold_avg_data.append(cold_avg)
                
            # Keep average data within plot period
            cutoff_time = current_time - timedelta(seconds=plot_period_seconds)
            while (len(self.avg_timestamps) > 0 and 
                   self.avg_timestamps[0] < cutoff_time):
                self.warm_avg_data.pop(0) if self.warm_avg_data else None
                self.cold_avg_data.pop(0) if self.cold_avg_data else None
                self.avg_timestamps.pop(0)
            
            # Safety limit for average data
            if len(self.avg_timestamps) > max_data_points:
                self.warm_avg_data = self.warm_avg_data[-max_data_points:]
                self.cold_avg_data = self.cold_avg_data[-max_data_points:]
                self.avg_timestamps = self.avg_timestamps[-max_data_points:]
            
            # Update individual channel lines
            for channel, line in self.temp_lines.items():
                if channel in self.temperature_data and len(self.temperature_data[channel]) > 0:
                    # Calculate relative time in seconds from start
                    if len(self.temperature_timestamps[channel]) > 0:
                        times = [(t - self.temperature_timestamps[channel][0]).total_seconds() 
                                for t in self.temperature_timestamps[channel]]
                        line.set_data(times, self.temperature_data[channel])
            
            # Update average lines
            if len(self.avg_timestamps) > 0 and len(self.warm_avg_data) > 0:
                avg_times = [(t - self.avg_timestamps[0]).total_seconds() for t in self.avg_timestamps]
                if len(self.warm_avg_data) > 0:
                    self.warm_avg_line.set_data(avg_times[:len(self.warm_avg_data)], self.warm_avg_data)
                if len(self.cold_avg_data) > 0:
                    self.cold_avg_line.set_data(avg_times[:len(self.cold_avg_data)], self.cold_avg_data)
            
            # Update fish temperature line (only if IR sensor is connected)
            if self.ir_sensor_controller:
                fish_temp = self.get_fish_temperature()
                if fish_temp is not None and (warm_temps or cold_temps):
                    # Only add fish temperature when we have average data (same timing)
                    self.fish_temp_data.append(fish_temp)
                    
                    # Keep fish temperature data synchronized with average data
                    # Remove excess data to match average data length
                    max_avg_length = max(len(self.warm_avg_data), len(self.cold_avg_data))
                    if len(self.fish_temp_data) > max_avg_length:
                        excess = len(self.fish_temp_data) - max_avg_length
                        self.fish_temp_data = self.fish_temp_data[excess:]
                    
                    # Update fish temperature line using average data timestamps for synchronization
                    if len(self.avg_timestamps) > 0 and len(self.fish_temp_data) > 0:
                        # Use the same time reference as average lines
                        avg_times = [(t - self.avg_timestamps[0]).total_seconds() for t in self.avg_timestamps]
                        # Align fish data with average timestamps (take the last N fish temperatures)
                        fish_data_aligned = self.fish_temp_data[-len(avg_times):] if len(self.fish_temp_data) >= len(avg_times) else self.fish_temp_data
                        time_data_aligned = avg_times[-len(fish_data_aligned):] if len(avg_times) >= len(fish_data_aligned) else avg_times
                        self.fish_temp_line.set_data(time_data_aligned, fish_data_aligned)
            
            # Adjust plot limits and update legend with current temperatures
            if any(len(data) > 0 for data in self.temperature_data.values()):
                self.ax.relim()
                self.ax.autoscale_view()
                
                # Update legend with current temperature values
                self.update_legend_with_temperatures()
                
                self.canvas.draw()
                
        except ValueError as e:
            # Handle array shape mismatches from data gaps
            if "shape mismatch" in str(e) or "broadcast" in str(e):
                print(f"Plot array mismatch detected: {e}")
                print("Attempting to synchronize data arrays...")
                self._synchronize_plot_arrays()
            else:
                print(f"ValueError in temperature plot: {e}")
        except Exception as e:
            print(f"Error updating temperature plot: {e}")
    
    def _synchronize_plot_arrays(self):
        """Synchronize plotting arrays to handle length mismatches from data gaps"""
        try:
            # Find minimum length across all temperature arrays
            min_length = float('inf')
            
            for ch in self.config.tc08_channels:
                if len(self.temperature_data[ch]) > 0:
                    min_length = min(min_length, len(self.temperature_data[ch]))
                    min_length = min(min_length, len(self.temperature_timestamps[ch]))
            
            # Check average arrays
            if len(self.warm_avg_data) > 0:
                min_length = min(min_length, len(self.warm_avg_data))
            if len(self.cold_avg_data) > 0:
                min_length = min(min_length, len(self.cold_avg_data))
            if len(self.avg_timestamps) > 0:
                min_length = min(min_length, len(self.avg_timestamps))
            
            # If we found valid arrays, trim all to minimum length
            if min_length != float('inf') and min_length > 0:
                for ch in self.config.tc08_channels:
                    self.temperature_data[ch] = self.temperature_data[ch][-min_length:]
                    self.temperature_timestamps[ch] = self.temperature_timestamps[ch][-min_length:]
                
                self.warm_avg_data = self.warm_avg_data[-min_length:]
                self.cold_avg_data = self.cold_avg_data[-min_length:]
                self.avg_timestamps = self.avg_timestamps[-min_length:]
                
                # Also trim fish temperature data
                if len(self.fish_temp_data) > min_length:
                    self.fish_temp_data = self.fish_temp_data[-min_length:]
                    self.fish_temp_timestamps = self.fish_temp_timestamps[-min_length:]
                
                print(f"Synchronized plot arrays to length {min_length}")
        except Exception as e:
            print(f"Error synchronizing plot arrays: {e}")
    
    def update_legend_with_temperatures(self):
        """Update plot legend to include current temperature values"""
        try:
            # Get current temperature values
            warm_avg = self.get_warm_side_average()
            cold_avg = self.get_cold_side_average()
            fish_temp = self.get_fish_temperature()
            
            # Get current control mode for status
            control_mode = self.control_status_var.get()
            if "Static Control:" in control_mode:
                control_short = "Static"
            elif "Dynamic Control:" in control_mode:
                control_short = "Dynamic"
            else:
                control_short = "Manual"
            
            # Build legend entries with temperature values
            legend_entries = []
            legend_handles = []
            
            # Individual channel lines (keep original names)
            warm_channels = [ch for i, ch in enumerate(self.config.tc08_channels) if i < 3]
            cold_channels = [ch for i, ch in enumerate(self.config.tc08_channels) if i >= 3]
            
            # Add warm side channels
            for channel in warm_channels:
                if channel in self.temp_lines:
                    channel_name = self.config.channel_names.get(channel, f"Ch{channel}")
                    legend_entries.append(f"W-Ch{channel}: {channel_name}")
                    legend_handles.append(self.temp_lines[channel])
            
            # Add cold side channels
            for channel in cold_channels:
                if channel in self.temp_lines:
                    channel_name = self.config.channel_names.get(channel, f"Ch{channel}")
                    legend_entries.append(f"C-Ch{channel}: {channel_name}")
                    legend_handles.append(self.temp_lines[channel])
            
            # Add average lines with current values
            if hasattr(self, 'warm_avg_line'):
                warm_text = f"WARM AVG: {warm_avg:.1f}°C" if warm_avg is not None else "WARM AVG: --°C"
                legend_entries.append(warm_text)
                legend_handles.append(self.warm_avg_line)
            
            if hasattr(self, 'cold_avg_line'):
                cold_text = f"COLD AVG: {cold_avg:.1f}°C" if cold_avg is not None else "COLD AVG: --°C"
                legend_entries.append(cold_text)
                legend_handles.append(self.cold_avg_line)
            
            # Add fish temperature with current value and mode
            if hasattr(self, 'fish_temp_line'):
                fish_text = f"FISH TEMP: {fish_temp:.1f}°C ({control_short})" if fish_temp is not None else f"FISH TEMP: --°C ({control_short})"
                legend_entries.append(fish_text)
                legend_handles.append(self.fish_temp_line)
            
            # Update the legend
            if legend_handles and legend_entries:
                self.ax.legend(legend_handles, legend_entries, bbox_to_anchor=(1.05, 1), loc='upper left')
                
        except Exception as e:
            pass  # Silently handle legend update errors
    
    def update_ir_visualization(self):
        """Update IR sensor and fish position visualization"""
        try:
            # Get current position (manual override takes precedence)
            current_position = self.get_current_fish_position()
            
            # Update position text
            position_names = {0: "Passage", 1: "Left (Cool)", 2: "Right (Warm)"}
            if self.persistent_manual_mode:
                position_text = f"Position: {position_names.get(current_position, 'Unknown')} (MANUAL - PERSISTENT)"
            elif self.manual_position_override is not None:
                position_text = f"Position: {position_names.get(current_position, 'Unknown')} (CORRECTING)"
            else:
                position_text = f"Position: {position_names.get(current_position, 'Unknown')}"
            self.fish_position_var.set(position_text)
            
            # Update fish position buttons with background colors
            self.update_position_buttons(current_position)
            
            # Update IR sensor states (only if IR sensor is connected)
            if self.ir_sensor_controller:
                ir_data = self.ir_sensor_controller.get_latest_data()
                # sensor_states[i]: True=clear/not interrupted (light gray), False=interrupted (red)
                for i, is_clear in enumerate(ir_data['sensor_states']):
                    if i < len(self.sensor_circles):
                        if is_clear:
                            color = 'lightgray'  # Sensor clear
                        else:
                            color = 'red'  # Sensor interrupted (fish present)
                        self.sensor_canvas.itemconfig(self.sensor_circles[i], fill=color)
            else:
                # IR sensor not connected - set all sensors to gray
                for i in range(10):
                    if i < len(self.sensor_circles):
                        self.sensor_canvas.itemconfig(self.sensor_circles[i], fill='lightgray')
                    
        except Exception as e:
            pass  # Silently handle IR visualization update errors
    
    def start_gui_updates(self):
        """Start periodic GUI updates"""
        self.update_gui()
    
    def clear_data(self):
        """Clear all collected data"""
        if messagebox.askyesno("Confirm", "Clear all collected data?"):
            for channel in self.temperature_data.keys():
                self.temperature_data[channel].clear()
                self.temperature_timestamps[channel].clear()
            
            # Clear average data
            self.warm_avg_data.clear()
            self.cold_avg_data.clear()
            self.avg_timestamps.clear()
            
            # Clear fish temperature data
            self.fish_temp_data.clear()
            
            # Clear plot
            for line in self.temp_lines.values():
                line.set_data([], [])
            if hasattr(self, 'warm_avg_line'):
                self.warm_avg_line.set_data([], [])
            if hasattr(self, 'cold_avg_line'):
                self.cold_avg_line.set_data([], [])
            if hasattr(self, 'fish_temp_line'):
                self.fish_temp_line.set_data([], [])
            if self.temp_lines:  # Only update if there are lines
                self.ax.relim()
                self.ax.autoscale_view()
                self.canvas.draw()
    
    def on_closing(self):
        """Handle application closing"""
        # Stop recording and automated control
        if self.recording_active:
            self.stop_recording()
        
        # Stop any automated temperature control and turn off all relays
        if self.static_control_active or self.dynamic_control_active:
            self.static_control_active = False
            self.dynamic_control_active = False
            if self.temp_control_controller:
                self.temp_control_controller.emergency_stop()
        
        # Ensure all relays are turned off at program exit
        if self.temp_control_controller:
            try:
                self.temp_control_controller.emergency_stop()  # Turn off all relays
            except:
                pass
        
        # Stop plotting and streaming
        self.plotting_active = False
        
        # Disconnect devices safely
        if self.tc08_controller:
            try:
                self.tc08_controller.stop_streaming()
                self.tc08_controller.disconnect()
            except:
                pass
            self.tc08_controller = None
        
        if self.ir_sensor_controller:
            try:
                self.ir_sensor_controller.disconnect()
            except:
                pass
            self.ir_sensor_controller = None
            
        if self.temp_control_controller:
            try:
                self.temp_control_controller.disconnect()
            except:
                pass
            self.temp_control_controller = None
        
        # Cancel timer
        if self.update_timer:
            self.root.after_cancel(self.update_timer)
        
        self.root.destroy()

def main():
    """Main application entry point"""
    root = tk.Tk()
    
    # Configure styles
    style = ttk.Style()
    style.configure("Emergency.TButton", foreground="red", font=('Arial', 10, 'bold'))
    
    app = ShuttleboxGUI(root)
    
    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    # Start the GUI
    root.mainloop()

if __name__ == "__main__":
    main()