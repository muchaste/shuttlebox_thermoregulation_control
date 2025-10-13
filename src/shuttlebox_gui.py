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
import json
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import queue

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
    temp_control_com_port: str
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
    
    def _process_message(self, message: str, timestamp: datetime):
        if message in ["0", "1", "2"]:
            self.fish_position = int(message)
            self.position_history.append({
                'timestamp': timestamp,
                'position': self.fish_position
            })
        elif message.startswith("SENSORS:"):
            # Expected format: "SENSORS:1010101010" (1=interrupted, 0=clear)
            sensor_data = message.split(":")[1]
            for i, state in enumerate(sensor_data[:10]):
                self.sensor_states[i] = (state == '1')

class TempControlController(SerialDevice):
    """Controller for temperature control Arduino"""
    def __init__(self, port: str):
        super().__init__(port)
        self.relay_states = {
            "HEAT": False,
            "COOL": False,
            "BHEAT": False,
            "BCOOL": False
        }
    
    def set_heating(self, enable: bool):
        command = "HEAT_ON" if enable else "HEAT_OFF"
        self.send_command(command)
    
    def set_cooling(self, enable: bool):
        command = "COOL_ON" if enable else "COOL_OFF"
        self.send_command(command)
    
    def set_buffer_heating(self, enable: bool):
        command = "BUFFER_HEAT_ON" if enable else "BUFFER_HEAT_OFF"
        self.send_command(command)
    
    def set_buffer_cooling(self, enable: bool):
        command = "BUFFER_COOL_ON" if enable else "BUFFER_COOL_OFF"
        self.send_command(command)
    
    def emergency_stop(self):
        self.send_command("ALL_OFF")
    
    def get_status(self):
        while not self.message_queue.empty():
            timestamp, message = self.message_queue.get()
            self._process_message(message)
        
        return self.relay_states.copy()
    
    def _process_message(self, message: str):
        if message.startswith("RELAY_"):
            parts = message.split(":")
            if len(parts) == 2:
                relay_type = parts[0].replace("RELAY_", "")
                state = parts[1] == "1"
                if relay_type in self.relay_states:
                    self.relay_states[relay_type] = state

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
            temp_control_com_port="COM6",
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
        
        # GUI update timer
        self.update_timer = None
        self.monitoring_active = False
        
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
        ttk.Label(tc08_frame, text="Device Ch#", font=('Arial', 8)).grid(row=1, column=0, padx=(20, 5), pady=2)
        ttk.Label(tc08_frame, text="Channel ID", font=('Arial', 8)).grid(row=1, column=1, padx=5, pady=2)
        ttk.Label(tc08_frame, text="Enable", font=('Arial', 8)).grid(row=1, column=2, padx=5, pady=2)
        
        # Channel configuration rows
        self.channel_vars = {}
        self.channel_id_vars = {}
        self.channel_enable_vars = {}
        
        default_channels = [1, 2, 3, 4, 5, 6]
        default_ids = ["w_inlet", "w_center", "w_buffer", "c_inlet", "c_center", "c_buffer"]
        
        current_row = 2
        
        # Add WARM SIDE label
        warm_label = ttk.Label(tc08_frame, text="WARM SIDE", font=("Arial", 9, "bold"), foreground="darkred")
        warm_label.grid(row=current_row, column=0, columnspan=3, sticky=tk.W, pady=(5, 0))
        current_row += 1
        
        # Add warm side channels (0-2)
        for i in range(3):
            channel = default_channels[i]
            channel_id = default_ids[i]
            
            # Channel number (device channel)
            self.channel_vars[i] = tk.StringVar(value=str(channel))
            channel_entry = ttk.Entry(tc08_frame, textvariable=self.channel_vars[i], width=8, justify=tk.CENTER)
            channel_entry.grid(row=current_row, column=0, padx=(20, 5), pady=1)
            
            # Channel ID (user-defined name)
            self.channel_id_vars[i] = tk.StringVar(value=channel_id)
            id_entry = ttk.Entry(tc08_frame, textvariable=self.channel_id_vars[i], width=15)
            id_entry.grid(row=current_row, column=1, padx=5, pady=1)
            
            # Enable checkbox
            self.channel_enable_vars[i] = tk.BooleanVar(value=True)
            enable_check = ttk.Checkbutton(tc08_frame, variable=self.channel_enable_vars[i])
            enable_check.grid(row=current_row, column=2, padx=5, pady=1)
            
            current_row += 1
        
        # Add COLD SIDE label
        cold_label = ttk.Label(tc08_frame, text="COLD SIDE", font=("Arial", 9, "bold"), foreground="darkblue")
        cold_label.grid(row=current_row, column=0, columnspan=3, sticky=tk.W, pady=(10, 0))
        current_row += 1
        
        # Add cold side channels (3-5)
        for i in range(3, 6):
            channel = default_channels[i]
            channel_id = default_ids[i]
            
            # Channel number (device channel)
            self.channel_vars[i] = tk.StringVar(value=str(channel))
            channel_entry = ttk.Entry(tc08_frame, textvariable=self.channel_vars[i], width=8, justify=tk.CENTER)
            channel_entry.grid(row=current_row, column=0, padx=(20, 5), pady=1)
            
            # Channel ID (user-defined name)
            self.channel_id_vars[i] = tk.StringVar(value=channel_id)
            id_entry = ttk.Entry(tc08_frame, textvariable=self.channel_id_vars[i], width=15)
            id_entry.grid(row=current_row, column=1, padx=5, pady=1)
            
            # Enable checkbox
            self.channel_enable_vars[i] = tk.BooleanVar(value=True)
            enable_check = ttk.Checkbutton(tc08_frame, variable=self.channel_enable_vars[i])
            enable_check.grid(row=current_row, column=2, padx=5, pady=1)
            
            current_row += 1
        
        # Sample interval
        ttk.Label(tc08_frame, text="Sample Interval (ms):").grid(row=current_row, column=0, sticky=tk.W, pady=(10, 2))
        self.tc08_interval_var = tk.StringVar(value="1000")
        ttk.Entry(tc08_frame, textvariable=self.tc08_interval_var, width=15).grid(row=current_row, column=1, padx=(0, 5), pady=(10, 2), sticky=tk.W)
        current_row += 1
        
        # TC-08 connect button
        self.tc08_connect_btn = ttk.Button(tc08_frame, text="Connect TC-08", command=self.connect_tc08)
        self.tc08_connect_btn.grid(row=current_row, column=0, columnspan=2, pady=10)
        
        # TC-08 status indicator (circle)
        self.tc08_status_canvas = tk.Canvas(tc08_frame, width=20, height=20, highlightthickness=0)
        self.tc08_status_canvas.grid(row=current_row, column=2, pady=5)
        self.tc08_status_circle = self.tc08_status_canvas.create_oval(2, 2, 18, 18, fill="red", outline="black")
        
        # Arduino Configuration Section
        arduino_frame = ttk.LabelFrame(parent, text="Arduino Configuration", padding="10")
        arduino_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Refresh COM ports button
        refresh_btn = ttk.Button(arduino_frame, text="Refresh Ports", command=self.refresh_com_ports)
        refresh_btn.grid(row=0, column=0, columnspan=2, pady=(0, 10), sticky=tk.W)
        
        # IR Sensor Arduino
        ttk.Label(arduino_frame, text="IR Sensor COM Port:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.ir_com_var = tk.StringVar()
        self.ir_com_combo = ttk.Combobox(arduino_frame, textvariable=self.ir_com_var, width=12, state="readonly")
        self.ir_com_combo.grid(row=1, column=1, padx=(10, 0))
        self.ir_connect_btn = ttk.Button(arduino_frame, text="Connect", command=self.connect_ir_sensor)
        self.ir_connect_btn.grid(row=1, column=2, padx=(10, 0))
        
        # IR status indicator (circle)
        self.ir_status_canvas = tk.Canvas(arduino_frame, width=20, height=20, highlightthickness=0)
        self.ir_status_canvas.grid(row=1, column=3, padx=(10, 0))
        self.ir_status_circle = self.ir_status_canvas.create_oval(2, 2, 18, 18, fill="red", outline="black")
        
        # Temperature Control Arduino  
        ttk.Label(arduino_frame, text="Temp Control COM Port:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.temp_com_var = tk.StringVar()
        self.temp_com_combo = ttk.Combobox(arduino_frame, textvariable=self.temp_com_var, width=12, state="readonly")
        self.temp_com_combo.grid(row=2, column=1, padx=(10, 0))
        self.temp_connect_btn = ttk.Button(arduino_frame, text="Connect", command=self.connect_temp_control)
        self.temp_connect_btn.grid(row=2, column=2, padx=(10, 0))
        
        # Temperature control status indicator (circle)
        self.temp_status_canvas = tk.Canvas(arduino_frame, width=20, height=20, highlightthickness=0)
        self.temp_status_canvas.grid(row=2, column=3, padx=(10, 0))
        self.temp_status_circle = self.temp_status_canvas.create_oval(2, 2, 18, 18, fill="red", outline="black")
        
        # Control Section
        control_frame = ttk.LabelFrame(parent, text="System Control", padding="10")
        control_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Monitoring controls
        self.start_btn = ttk.Button(control_frame, text="Start Monitoring", command=self.start_monitoring, state=tk.DISABLED)
        self.start_btn.pack(fill=tk.X, pady=2)
        
        self.stop_btn = ttk.Button(control_frame, text="Stop Monitoring", command=self.stop_monitoring, state=tk.DISABLED)
        self.stop_btn.pack(fill=tk.X, pady=2)
        
        # Temperature control buttons
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        ttk.Label(control_frame, text="Manual Temperature Control:").pack()
        
        temp_btn_frame = ttk.Frame(control_frame)
        temp_btn_frame.pack(fill=tk.X, pady=5)
        
        self.heat_btn = ttk.Button(temp_btn_frame, text="Heat ON", command=lambda: self.manual_temp_control('heat', True))
        self.heat_btn.pack(side=tk.LEFT, padx=2)
        
        self.heat_off_btn = ttk.Button(temp_btn_frame, text="Heat OFF", command=lambda: self.manual_temp_control('heat', False))
        self.heat_off_btn.pack(side=tk.LEFT, padx=2)
        
        temp_btn_frame2 = ttk.Frame(control_frame)
        temp_btn_frame2.pack(fill=tk.X, pady=5)
        
        self.cool_btn = ttk.Button(temp_btn_frame2, text="Cool ON", command=lambda: self.manual_temp_control('cool', True))
        self.cool_btn.pack(side=tk.LEFT, padx=2)
        
        self.cool_off_btn = ttk.Button(temp_btn_frame2, text="Cool OFF", command=lambda: self.manual_temp_control('cool', False))
        self.cool_off_btn.pack(side=tk.LEFT, padx=2)
        
        self.emergency_btn = ttk.Button(control_frame, text="EMERGENCY STOP", command=self.emergency_stop)
        self.emergency_btn.pack(fill=tk.X, pady=10)
        self.emergency_btn.configure(style="Emergency.TButton")
        
        # Data controls
        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        self.export_btn = ttk.Button(control_frame, text="Export Data", command=self.export_data)
        self.export_btn.pack(fill=tk.X, pady=2)
        
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
        self.ax.text(0.5, 0.5, 'Connect TC-08 to start monitoring', 
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
                                          color='red', linewidth=3, alpha=1.0)
        self.cold_avg_line, = self.ax.plot([], [], label='COLD AVERAGE', 
                                          color='blue', linewidth=3, alpha=1.0)
        
        # Initialize average data storage
        self.warm_avg_data = []
        self.cold_avg_data = []
        self.avg_timestamps = []
        
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
        
        position_frame = ttk.Frame(fish_frame)
        position_frame.pack(expand=True, fill=tk.BOTH, padx=10, pady=10)
        
        # Position boxes
        self.position_labels = {}
        positions = ['LEFT', 'PASSAGE', 'RIGHT']
        for i, pos in enumerate(positions):
            frame = ttk.Frame(position_frame, relief=tk.RAISED, borderwidth=2)
            frame.grid(row=0, column=i, padx=5, pady=5, sticky="nsew")
            
            label = ttk.Label(frame, text=pos, font=('Arial', 12, 'bold'))
            label.pack(expand=True, fill=tk.BOTH, padx=20, pady=20)
            
            self.position_labels[i+1 if i > 0 else 0] = frame  # Map to position codes
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
        """Refresh available COM ports in dropdown menus"""
        try:
            ports = serial.tools.list_ports.comports()
            port_names = [port.device for port in ports]
            
            # Update dropdown values
            self.ir_com_combo['values'] = port_names
            self.temp_com_combo['values'] = port_names
            
            # Set default values if available
            if port_names:
                if not self.ir_com_var.get():
                    self.ir_com_var.set(port_names[0])
                if not self.temp_com_var.get():
                    self.temp_com_var.set(port_names[-1] if len(port_names) > 1 else port_names[0])
        except Exception as e:
            messagebox.showerror("Error", f"Failed to refresh COM ports: {e}")
    
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
                # Update temperature data storage for new channels
                self.temperature_data = {ch: [] for ch in channels}
                self.temperature_timestamps = {ch: [] for ch in channels}
                
                # Update plot with new channel configuration
                self.update_plot_configuration()
                
                self.tc08_status_canvas.itemconfig(self.tc08_status_circle, fill="green")
                self.tc08_connect_btn.config(text="Disconnect TC-08", command=self.disconnect_tc08)
                
                # Show connection summary
                channel_summary = ", ".join([f"Ch{ch}({channel_names[ch]})" for ch in channels])
                messagebox.showinfo("Success", f"TC-08 connected successfully!\nChannels: {channel_summary}")
                self.update_monitoring_button_state()  # Check if all devices connected
            else:
                self.tc08_status_canvas.itemconfig(self.tc08_status_circle, fill="red")
                messagebox.showerror("Error", "Failed to connect to TC-08")
                
        except Exception as e:
            messagebox.showerror("Error", f"TC-08 connection error: {str(e)}")
    
    def disconnect_tc08(self):
        """Disconnect from TC-08"""
        if self.tc08_controller:
            self.tc08_controller.disconnect()
            self.tc08_controller = None
        
        self.tc08_status_canvas.itemconfig(self.tc08_status_circle, fill="red")
        self.tc08_connect_btn.config(text="Connect TC-08", command=self.connect_tc08)
        self.update_monitoring_button_state()  # Update monitoring button state
    
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
        """Connect to temperature control Arduino"""
        try:
            port = self.temp_com_var.get()
            if not port:
                messagebox.showerror("Error", "Please select a COM port")
                return
                
            self.temp_control_controller = TempControlController(port)
            
            if self.temp_control_controller.connect("TEMP_CTRL_READY"):
                self.temp_status_canvas.itemconfig(self.temp_status_circle, fill="green")
                self.temp_connect_btn.config(text="Disconnect", command=self.disconnect_temp_control)
                self._temp_warning_shown = False  # Reset warning flag
                self.update_monitoring_button_state()  # Check if all devices connected
            else:
                self.temp_status_canvas.itemconfig(self.temp_status_circle, fill="red")
                messagebox.showerror("Error", f"Wrong Arduino type on {port}.\nExpected Temperature Controller (TEMP_CTRL_READY)")
                self.temp_control_controller = None
                
        except Exception as e:
            messagebox.showerror("Error", f"Temperature control connection error: {str(e)}")
            self.temp_control_controller = None
    
    def disconnect_temp_control(self):
        """Disconnect from temperature control Arduino"""
        if self.temp_control_controller:
            self.temp_control_controller.disconnect()
            self.temp_control_controller = None
        
        self.temp_status_canvas.itemconfig(self.temp_status_circle, fill="red")
        self.temp_connect_btn.config(text="Connect", command=self.connect_temp_control)
        self.update_monitoring_button_state()  # Update monitoring button state
    
    def start_monitoring(self):
        """Start system monitoring"""
        if not self.tc08_controller:
            messagebox.showerror("Error", "TC-08 not connected")
            return
        
        if not self.tc08_controller.start_streaming():
            messagebox.showerror("Error", "Failed to start temperature monitoring")
            return
        
        self.monitoring_active = True
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        
        # Disable all connection controls during monitoring
        self._set_connection_controls_state(tk.DISABLED)
        
        # Clear existing data
        for channel in self.config.tc08_channels:
            self.temperature_data[channel].clear()
            self.temperature_timestamps[channel].clear()
    
    def stop_monitoring(self):
        """Stop system monitoring"""
        if self.tc08_controller:
            self.tc08_controller.stop_streaming()
        
        self.monitoring_active = False
        self.stop_btn.config(state=tk.DISABLED)
        
        # Re-enable all connection controls after monitoring stops
        self._set_connection_controls_state(tk.NORMAL)
        
        # Update monitoring button state (only enable if all devices connected)
        self.update_monitoring_button_state()
    
    def manual_temp_control(self, control_type: str, enable: bool):
        """Manual temperature control"""
        if not self.temp_control_controller:
            messagebox.showwarning("Warning", "Temperature control not connected")
            return
        
        if control_type == 'heat':
            self.temp_control_controller.set_heating(enable)
        elif control_type == 'cool':
            self.temp_control_controller.set_cooling(enable)
    
    def emergency_stop(self):
        """Emergency stop all temperature control"""
        if self.temp_control_controller:
            self.temp_control_controller.emergency_stop()
        messagebox.showinfo("Emergency Stop", "All temperature control systems stopped!")
    
    def update_gui(self):
        """Update GUI with latest data (called periodically)"""
        try:
            # Update temperature data and plot
            if self.monitoring_active and self.tc08_controller:
                self.update_temperature_plot()
            
            # Update IR sensor and fish position
            if self.ir_sensor_controller:
                self.update_ir_visualization()
            
            # Check Arduino connection health
            self.check_arduino_connections()
            
        except Exception as e:
            print(f"GUI update error: {e}")
        
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
                    if self.monitoring_active:
                        messagebox.showerror("Connection Lost", 
                                           "IR Sensor Arduino connection lost!\nStopping monitoring for safety.")
                        self.stop_monitoring()
            else:
                self._ir_warning_shown = False
                if self.ir_sensor_controller.connected:
                    self.ir_status_canvas.itemconfig(self.ir_status_circle, fill="green")
        
        # Check temperature control connection  
        if self.temp_control_controller:
            if not self.temp_control_controller.is_connection_healthy():
                if not hasattr(self, '_temp_warning_shown') or not self._temp_warning_shown:
                    self.temp_status_canvas.itemconfig(self.temp_status_circle, fill="orange")
                    self._temp_warning_shown = True
                    if self.monitoring_active:
                        messagebox.showerror("Connection Lost",
                                           "Temperature Control Arduino connection lost!\nStopping monitoring for safety.")
                        self.stop_monitoring()
            else:
                self._temp_warning_shown = False
                if self.temp_control_controller.connected:
                    self.temp_status_canvas.itemconfig(self.temp_status_circle, fill="green")
    
    def update_monitoring_button_state(self):
        """Enable Start Monitoring button only when all three devices are connected"""
        all_connected = (
            self.tc08_controller is not None and 
            self.ir_sensor_controller is not None and 
            self.temp_control_controller is not None and
            self.tc08_controller.connected and
            self.ir_sensor_controller.connected and 
            self.temp_control_controller.connected
        )
        
        if all_connected and not self.monitoring_active:
            self.start_btn.config(state=tk.NORMAL)
        else:
            self.start_btn.config(state=tk.DISABLED)
    
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
        
        # Sample interval entry
        try:
            for widget in self.root.winfo_children():
                self._find_and_set_entry_state(widget, self.tc08_interval_var, state)
        except:
            pass
        
        # COM port combo boxes
        try:
            com_state = "readonly" if state == tk.DISABLED else "readonly"  # Keep readonly for dropdowns
            self.ir_com_combo.config(state=com_state if state == tk.NORMAL else tk.DISABLED)
            self.temp_com_combo.config(state=com_state if state == tk.NORMAL else tk.DISABLED)
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
    
    def update_temperature_plot(self):
        """Update temperature plot with new data and warm/cold averages"""
        try:
            readings = self.tc08_controller.read_temperatures()
            current_time = datetime.now()
            
            # Add new data points
            for channel, reading in readings.items():
                if channel in self.temperature_data:  # Only update configured channels
                    if not reading.error and reading.temperature is not None:
                        self.temperature_data[channel].append(reading.temperature)
                        self.temperature_timestamps[channel].append(current_time)
                        
                        # Keep only last 100 points
                        if len(self.temperature_data[channel]) > 100:
                            self.temperature_data[channel] = self.temperature_data[channel][-100:]
                            self.temperature_timestamps[channel] = self.temperature_timestamps[channel][-100:]
            
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
                
            # Keep only last 100 points for averages
            if len(self.warm_avg_data) > 100:
                self.warm_avg_data = self.warm_avg_data[-100:]
                self.cold_avg_data = self.cold_avg_data[-100:]
                self.avg_timestamps = self.avg_timestamps[-100:]
            
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
            
            # Adjust plot limits
            if any(len(data) > 0 for data in self.temperature_data.values()):
                self.ax.relim()
                self.ax.autoscale_view()
                self.canvas.draw()
                
        except Exception as e:
            print(f"Temperature plot update error: {e}")
    
    def update_ir_visualization(self):
        """Update IR sensor and fish position visualization"""
        try:
            ir_data = self.ir_sensor_controller.get_latest_data()
            
            # Update fish position
            for pos, frame in self.position_labels.items():
                if pos == ir_data['fish_position']:
                    frame.config(relief=tk.RAISED, borderwidth=3)
                    # Change background color (simplified - would need more complex styling)
                else:
                    frame.config(relief=tk.RAISED, borderwidth=1)
            
            # Update IR sensor states
            for i, interrupted in enumerate(ir_data['sensor_states']):
                if i in self.sensor_circles:
                    color = 'red' if interrupted else 'lightgray'
                    self.sensor_canvas.itemconfig(self.sensor_circles[i], fill=color)
                    
        except Exception as e:
            print(f"IR visualization update error: {e}")
    
    def start_gui_updates(self):
        """Start periodic GUI updates"""
        self.update_gui()
    
    def export_data(self):
        """Export collected data to CSV file"""
        if not any(len(data) > 0 for data in self.temperature_data.values()):
            messagebox.showwarning("Warning", "No data to export")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            title="Export Temperature Data"
        )
        
        if filename:
            try:
                import csv
                with open(filename, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    
                    # Write header
                    header = ['Timestamp'] + [f'Ch{ch}_{self.config.channel_names.get(ch, f"Channel_{ch}")}' 
                                            for ch in self.config.tc08_channels] + ['Warm_Average', 'Cold_Average']
                    writer.writerow(header)
                    
                    # Find maximum data length
                    max_length = max(len(data) for data in self.temperature_data.values() if data)
                    
                    # Write data rows
                    for i in range(max_length):
                        row = []
                        
                        # Use timestamp from first channel that has data at this index
                        timestamp = None
                        for ch in self.config.tc08_channels:
                            if ch in self.temperature_timestamps and i < len(self.temperature_timestamps[ch]):
                                timestamp = self.temperature_timestamps[ch][i]
                                break
                        
                        row.append(timestamp.strftime('%Y-%m-%d %H:%M:%S.%f') if timestamp else '')
                        
                        # Add temperature data
                        for ch in self.config.tc08_channels:
                            if ch in self.temperature_data and i < len(self.temperature_data[ch]):
                                row.append(f"{self.temperature_data[ch][i]:.4f}")
                            else:
                                row.append('')
                        
                        # Add warm and cold averages
                        if i < len(self.warm_avg_data):
                            row.append(f"{self.warm_avg_data[i]:.4f}")
                        else:
                            row.append('')
                            
                        if i < len(self.cold_avg_data):
                            row.append(f"{self.cold_avg_data[i]:.4f}")
                        else:
                            row.append('')
                        
                        writer.writerow(row)
                
                messagebox.showinfo("Success", f"Data exported to {filename}")
                
            except Exception as e:
                messagebox.showerror("Error", f"Export failed: {str(e)}")
    
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
            
            # Clear plot
            for line in self.temp_lines.values():
                line.set_data([], [])
            if hasattr(self, 'warm_avg_line'):
                self.warm_avg_line.set_data([], [])
            if hasattr(self, 'cold_avg_line'):
                self.cold_avg_line.set_data([], [])
            if self.temp_lines:  # Only update if there are lines
                self.ax.relim()
                self.ax.autoscale_view()
                self.canvas.draw()
    
    def on_closing(self):
        """Handle application closing"""
        # Stop monitoring
        if self.monitoring_active:
            self.stop_monitoring()
        
        # Disconnect devices
        if self.tc08_controller:
            self.disconnect_tc08()
        if self.ir_sensor_controller:
            self.disconnect_ir_sensor()
        if self.temp_control_controller:
            self.disconnect_temp_control()
        
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