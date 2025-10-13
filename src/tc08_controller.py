"""
TC-08 Temperature Data Acquisition System
Scientific-grade thermocouple interface for shuttlebox thermoregulation experiments

Key Features:
- Multi-channel K-type thermocouple support
- Configurable sampling rates and channel assignments
- Thread-safe data acquisition
- Comprehensive error handling and validation
- Clean API for GUI integration

Author: Shuttlebox Control System
Version: 1.0
"""

import ctypes
import time
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Callable
from datetime import datetime
import logging
from picosdk.usbtc08 import usbtc08 as tc08
from picosdk.functions import assert_pico2000_ok

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class TemperatureReading:
    """Data structure for a single temperature measurement"""
    channel: int
    temperature: float
    timestamp: datetime
    readings_count: int
    overflow: bool
    error: Optional[str] = None

@dataclass
class TC08Config:
    """Configuration parameters for TC-08 device"""
    channels: List[int]
    sample_interval_ms: int = 1000  # Sampling interval in milliseconds
    thermocouple_type: str = 'K'    # K-type thermocouple
    units: int = 0                   # 0=Celsius, 1=Fahrenheit, 2=Kelvin
    fill_missing: bool = True        # Fill missing readings with previous values
    max_buffer_size: int = 100       # Maximum readings per channel call

class TC08Controller:
    """
    Main controller class for TC-08 temperature data acquisition
    Designed for integration with GUI applications
    """
    
    def __init__(self, config: TC08Config):
        self.config = config
        self.chandle = ctypes.c_int16()
        self.status = {}
        self.connected = False
        self.streaming = False
        self._lock = threading.Lock()
        
        # Thermocouple type mapping
        self._tc_types = {
            'B': ctypes.c_int8(66), 'E': ctypes.c_int8(69), 'J': ctypes.c_int8(74),
            'K': ctypes.c_int8(75), 'N': ctypes.c_int8(78), 'R': ctypes.c_int8(82),
            'S': ctypes.c_int8(83), 'T': ctypes.c_int8(84)
        }
        
        # Validate configuration
        self._validate_config()
        
    def _validate_config(self):
        """Validate configuration parameters"""
        if not self.config.channels:
            raise ValueError("At least one channel must be specified")
        
        if not all(1 <= ch <= 8 for ch in self.config.channels):
            raise ValueError("Channel numbers must be between 1 and 8")
        
        if self.config.thermocouple_type not in self._tc_types:
            raise ValueError(f"Unsupported thermocouple type: {self.config.thermocouple_type}")
        
        if self.config.sample_interval_ms < 100:
            raise ValueError("Sample interval must be at least 100ms")
    
    def connect(self) -> bool:
        """
        Connect to TC-08 device and configure channels
        Returns: True if successful, False otherwise
        """
        with self._lock:
            try:
                if self.connected:
                    logger.warning("Already connected to TC-08")
                    return True
                
                # Open device
                self.status["open_unit"] = tc08.usb_tc08_open_unit()
                assert_pico2000_ok(self.status["open_unit"])
                self.chandle = self.status["open_unit"]
                logger.info(f"TC-08 device opened with handle: {self.chandle}")
                
                # Set mains rejection (50Hz for EU, 60Hz for US)
                self.status["set_mains"] = tc08.usb_tc08_set_mains(self.chandle, 0)  # 0=50Hz
                assert_pico2000_ok(self.status["set_mains"])
                
                # Configure channels
                tc_type = self._tc_types[self.config.thermocouple_type]
                for channel in self.config.channels:
                    status_key = f"set_channel_{channel}"
                    self.status[status_key] = tc08.usb_tc08_set_channel(self.chandle, channel, tc_type)
                    assert_pico2000_ok(self.status[status_key])
                    logger.info(f"Configured channel {channel} for {self.config.thermocouple_type}-type thermocouple")
                
                # Get and validate minimum sampling interval
                self.status["get_minimum_interval_ms"] = tc08.usb_tc08_get_minimum_interval_ms(self.chandle)
                assert_pico2000_ok(self.status["get_minimum_interval_ms"])
                
                min_interval = self.status["get_minimum_interval_ms"]
                if self.config.sample_interval_ms < min_interval:
                    logger.warning(f"Requested interval {self.config.sample_interval_ms}ms is below minimum {min_interval}ms")
                    self.config.sample_interval_ms = min_interval
                
                self.connected = True
                logger.info(f"TC-08 connected successfully. Channels: {self.config.channels}, "
                           f"Min interval: {min_interval}ms, Using: {self.config.sample_interval_ms}ms")
                
                return True
                
            except Exception as e:
                logger.error(f"Failed to connect TC-08: {e}")
                self._cleanup_connection()
                return False
    
    def start_streaming(self) -> bool:
        """
        Start streaming mode for continuous temperature acquisition
        Returns: True if successful, False otherwise
        """
        with self._lock:
            try:
                if not self.connected:
                    logger.error("Cannot start streaming: device not connected")
                    return False
                
                if self.streaming:
                    logger.warning("Streaming already active")
                    return True
                
                # Start streaming mode
                self.status["run"] = tc08.usb_tc08_run(self.chandle, self.config.sample_interval_ms)
                assert_pico2000_ok(self.status["run"])
                
                # Allow device to stabilize
                time.sleep(2.0)
                
                self.streaming = True
                logger.info("TC-08 streaming started")
                return True
                
            except Exception as e:
                logger.error(f"Failed to start streaming: {e}")
                return False
    
    def stop_streaming(self) -> bool:
        """
        Stop streaming mode
        Returns: True if successful, False otherwise
        """
        with self._lock:
            try:
                if not self.streaming:
                    return True
                
                self.status["stop"] = tc08.usb_tc08_stop(self.chandle)
                assert_pico2000_ok(self.status["stop"])
                
                self.streaming = False
                logger.info("TC-08 streaming stopped")
                return True
                
            except Exception as e:
                logger.error(f"Failed to stop streaming: {e}")
                return False
    
    def read_temperatures(self) -> Dict[int, TemperatureReading]:
        """
        Read current temperatures from all configured channels
        Returns: Dictionary mapping channel number to TemperatureReading
        """
        if not self.connected:
            raise RuntimeError("Device not connected")
        
        if not self.streaming:
            raise RuntimeError("Device not in streaming mode")
        
        readings = {}
        timestamp = datetime.now()
        
        for channel in self.config.channels:
            reading = self._read_single_channel(channel, timestamp)
            readings[channel] = reading
        
        return readings
    
    def _read_single_channel(self, channel: int, timestamp: datetime) -> TemperatureReading:
        """Read temperature from a single channel"""
        try:
            # Prepare buffers
            temp_buffer = (ctypes.c_float * self.config.max_buffer_size)()
            times_ms_buffer = (ctypes.c_int32 * self.config.max_buffer_size)()
            overflow = ctypes.c_int16()
            
            # Read channel data
            readings_count = tc08.usb_tc08_get_temp(
                self.chandle,
                ctypes.byref(temp_buffer),
                ctypes.byref(times_ms_buffer),
                self.config.max_buffer_size,
                ctypes.byref(overflow),
                channel,
                self.config.units,
                1 if self.config.fill_missing else 0
            )
            
            if readings_count > 0:
                # Get most recent reading
                temperature = float(temp_buffer[0])
                return TemperatureReading(
                    channel=channel,
                    temperature=temperature,
                    timestamp=timestamp,
                    readings_count=readings_count,
                    overflow=overflow.value > 0
                )
            else:
                return TemperatureReading(
                    channel=channel,
                    temperature=float('nan'),
                    timestamp=timestamp,
                    readings_count=0,
                    overflow=False,
                    error="No readings available"
                )
                
        except Exception as e:
            logger.error(f"Error reading channel {channel}: {e}")
            return TemperatureReading(
                channel=channel,
                temperature=float('nan'),
                timestamp=timestamp,
                readings_count=0,
                overflow=False,
                error=str(e)
            )
    
    def get_device_info(self) -> Dict[str, any]:
        """Get device information and status"""
        return {
            'connected': self.connected,
            'streaming': self.streaming,
            'channels': self.config.channels,
            'sample_interval_ms': self.config.sample_interval_ms,
            'thermocouple_type': self.config.thermocouple_type,
            'min_interval_ms': self.status.get("get_minimum_interval_ms", None),
            'handle': int(self.chandle) if self.connected else None
        }
    
    def disconnect(self) -> bool:
        """
        Disconnect from TC-08 device
        Returns: True if successful, False otherwise
        """
        with self._lock:
            try:
                if not self.connected:
                    return True
                
                # Stop streaming if active
                if self.streaming:
                    self.stop_streaming()
                
                # Close device
                self.status["close_unit"] = tc08.usb_tc08_close_unit(self.chandle)
                assert_pico2000_ok(self.status["close_unit"])
                
                self._cleanup_connection()
                logger.info("TC-08 disconnected successfully")
                return True
                
            except Exception as e:
                logger.error(f"Error during disconnect: {e}")
                self._cleanup_connection()
                return False
    
    def _cleanup_connection(self):
        """Clean up connection state"""
        self.connected = False
        self.streaming = False
        self.chandle = ctypes.c_int16()
        self.status.clear()
    
    def __enter__(self):
        """Context manager entry"""
        if not self.connect():
            raise RuntimeError("Failed to connect to TC-08")
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()


class TC08DataLogger:
    """
    Data logging and analysis helper for TC-08 temperature readings
    Designed for scientific data collection and analysis
    """
    
    def __init__(self):
        self.data_history = []
        self._lock = threading.Lock()
    
    def log_readings(self, readings: Dict[int, TemperatureReading]):
        """Add temperature readings to data history"""
        with self._lock:
            for channel, reading in readings.items():
                if reading.error is None:  # Only log valid readings
                    self.data_history.append({
                        'timestamp': reading.timestamp,
                        'channel': channel,
                        'temperature': reading.temperature,
                        'readings_count': reading.readings_count,
                        'overflow': reading.overflow
                    })
    
    def get_latest_by_channel(self, channel: int, count: int = 1) -> List[Dict]:
        """Get the most recent readings for a specific channel"""
        with self._lock:
            channel_data = [d for d in self.data_history if d['channel'] == channel]
            return channel_data[-count:] if channel_data else []
    
    def get_data_range(self, start_time: datetime, end_time: datetime) -> List[Dict]:
        """Get all readings within a time range"""
        with self._lock:
            return [d for d in self.data_history 
                   if start_time <= d['timestamp'] <= end_time]
    
    def export_csv(self, filename: str, channels: Optional[List[int]] = None):
        """Export data to CSV file"""
        import csv
        
        with self._lock:
            data_to_export = self.data_history
            if channels:
                data_to_export = [d for d in data_to_export if d['channel'] in channels]
            
            with open(filename, 'w', newline='') as csvfile:
                if not data_to_export:
                    return
                
                fieldnames = ['timestamp', 'channel', 'temperature', 'readings_count', 'overflow']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(data_to_export)
    
    def clear_history(self):
        """Clear all stored data"""
        with self._lock:
            self.data_history.clear()


# Example usage and testing
if __name__ == "__main__":
    # Example configuration for shuttlebox system
    config = TC08Config(
        channels=[1, 2, 3, 4, 5, 6],  # All 6 shuttlebox channels
        sample_interval_ms=1000,       # 1 second sampling
        thermocouple_type='K'          # K-type thermocouples
    )
    
    # Initialize data logger
    logger_instance = TC08DataLogger()
    
    try:
        # Use context manager for automatic connection handling
        with TC08Controller(config) as tc08_device:
            print("Device info:", tc08_device.get_device_info())
            
            # Start streaming
            if tc08_device.start_streaming():
                print("Starting temperature monitoring...")
                
                # Collect data for 10 seconds
                for i in range(10):
                    readings = tc08_device.read_temperatures()
                    logger_instance.log_readings(readings)
                    
                    print(f"\n--- Reading {i+1} ---")
                    for channel, reading in readings.items():
                        if reading.error:
                            print(f"Ch{channel}: ERROR - {reading.error}")
                        else:
                            print(f"Ch{channel}: {reading.temperature:.2f}°C "
                                 f"({reading.readings_count} samples)")
                    
                    time.sleep(1)
                
                # Export data
                logger_instance.export_csv("temperature_data.csv")
                print("\nData exported to temperature_data.csv")
            
    except Exception as e:
        print(f"Error: {e}")