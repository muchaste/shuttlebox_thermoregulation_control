"""
TC-08 Temperature Data Acquisition System
Scientific-grade thermocouple interface for shuttlebox thermoregulation experiments

Key Features:
- Multi-channel K-type thermocouple support
- Configurable sampling rates and channel assignments
- Thread-safe data acquisition
- Comprehensive error handling and validation
- Clean API for GUI integration

Author: Stefan Mucha, Claude Sonnet 4.0
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
import json
import os
from pathlib import Path

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
    """Main controller class for TC-08 temperature data acquisition"""
    
    def __init__(self, config: TC08Config):
        self.config = config
        self.chandle = ctypes.c_int16()
        self.status = {}
        self.connected = False
        self.streaming = False
        self._lock = threading.Lock()
        
        # Add health monitoring
        self.health_monitor = TC08HealthMonitor()
        self._health_logging_timer = None
        self._health_logging_active = False
        
        # Track consecutive read failures for graduated response
        self.consecutive_read_failures = 0
        self.total_read_failures = 0
        self.last_successful_read = None
        self.last_failed_read = None
        
        # Thermocouple type mapping
        self._tc_types = {
            'B': ctypes.c_int8(66), 'E': ctypes.c_int8(69), 'J': ctypes.c_int8(74),
            'K': ctypes.c_int8(75), 'N': ctypes.c_int8(78), 'R': ctypes.c_int8(82),
            'S': ctypes.c_int8(83), 'T': ctypes.c_int8(84)
        }
        
        # TC-08 error code mapping (from programmer's guide)
        self._error_codes = {
            -1: "INVALID_HANDLE",
            0: "OK - No error occurred",
            1: "OS_NOT_SUPPORTED - Driver does not support current OS",
            2: "NO_CHANNELS_SET - Call to usb_tc08_set_channel required",
            3: "INVALID_PARAMETER - One or more function arguments invalid",
            4: "VARIANT_NOT_SUPPORTED - Hardware version not supported",
            5: "INCORRECT_MODE - Incompatible mix of legacy/non-legacy functions",
            6: "ENUMERATION_INCOMPLETE - Background enumeration in progress",
            7: "NOT_RESPONDING - Cannot get reply from USB TC-08",
            8: "FW_FAIL - Unable to download firmware",
            9: "CONFIG_FAIL - Missing or corrupted EEPROM",
            10: "NOT_FOUND - Cannot find enumerated device",
            11: "THREAD_FAIL - Threading function failed",
            12: "PIPE_INFO_FAIL - Cannot get USB pipe information",
            13: "NOT_CALIBRATED - No calibration date found",
            14: "PICOPP_TOO_OLD - Old picopp.sys driver found",
            15: "COMMUNICATION - PC lost communication with device"
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
    
    def get_last_error(self) -> Dict[str, any]:
        """Get last error from TC-08 device with detailed diagnostic information
        
        Returns:
            Dictionary with error_code, error_name, and error_description
        """
        try:
            if not self.connected or self.chandle.value == 0:
                return {
                    'error_code': -1,
                    'error_name': 'INVALID_HANDLE',
                    'error_description': 'Device not connected or invalid handle'
                }
            
            # Call TC-08 API to get last error
            error_code = tc08.usb_tc08_get_last_error(self.chandle.value)
            
            # Look up error description
            error_description = self._error_codes.get(error_code, f"UNKNOWN_ERROR_{error_code}")
            
            # Extract just the error name (before the dash)
            error_name = error_description.split(' - ')[0] if ' - ' in error_description else error_description
            
            error_info = {
                'error_code': error_code,
                'error_name': error_name,
                'error_description': error_description,
                'timestamp': datetime.now().isoformat()
            }
            
            # Log to health monitor if error is not OK
            if error_code != 0:
                logger.warning(f"TC-08 Error {error_code}: {error_description}")
                self.health_monitor.log_error(
                    f"TC-08 API Error: {error_description}",
                    error_info
                )
            
            return error_info
            
        except Exception as e:
            logger.error(f"Failed to get TC-08 last error: {e}")
            return {
                'error_code': -999,
                'error_name': 'EXCEPTION',
                'error_description': f'Exception during get_last_error: {str(e)}',
                'timestamp': datetime.now().isoformat()
            }
    
    def connect(self) -> bool:
        """Connect to TC-08 device and configure channels"""
        with self._lock:
            try:
                if self.connected:
                    logger.warning("Already connected to TC-08")
                    return True
                
                # Log connection attempt
                self.health_monitor.log_connection_event("connect_attempt", True, {
                    "channels": self.config.channels,
                    "sample_interval_ms": self.config.sample_interval_ms
                })
                
                # Open device
                self.status["open_unit"] = tc08.usb_tc08_open_unit()
                assert_pico2000_ok(self.status["open_unit"])
                self.chandle.value = self.status["open_unit"]
                
                # Log successful device open
                self.health_monitor.log_connection_event("device_opened", True, {
                    "handle_value": int(self.chandle.value)
                })
                
                logger.info(f"TC-08 device opened with handle: {self.chandle.value}")
                
                # Set mains rejection (50Hz for EU, 60Hz for US)
                self.status["set_mains"] = tc08.usb_tc08_set_mains(self.chandle.value, 0)
                assert_pico2000_ok(self.status["set_mains"])
                
                # Configure channels
                tc_type = self._tc_types[self.config.thermocouple_type]
                for channel in self.config.channels:
                    status_key = f"set_channel_{channel}"
                    self.status[status_key] = tc08.usb_tc08_set_channel(self.chandle.value, channel, tc_type)
                    assert_pico2000_ok(self.status[status_key])
                
                # Get and validate minimum sampling interval
                self.status["get_minimum_interval_ms"] = tc08.usb_tc08_get_minimum_interval_ms(self.chandle.value)
                assert_pico2000_ok(self.status["get_minimum_interval_ms"])
                
                min_interval = self.status["get_minimum_interval_ms"]
                if self.config.sample_interval_ms < min_interval:
                    logger.warning(f"Requested interval {self.config.sample_interval_ms}ms is below minimum {min_interval}ms")
                    self.config.sample_interval_ms = min_interval
                
                self.connected = True
                
                # Reset failure counters on successful connection
                self.consecutive_read_failures = 0
                self.last_successful_read = datetime.now()
                
                # Log successful connection
                self.health_monitor.log_connection_event("connect_success", True, {
                    "min_interval_ms": min_interval,
                    "actual_interval_ms": self.config.sample_interval_ms
                })
                
                logger.info(f"TC-08 connected successfully. Channels: {self.config.channels}")
                return True
                
            except Exception as e:
                # Log connection failure with full details
                self.health_monitor.log_connection_event("connect_failed", False, {
                    "error": str(e),
                    "status_codes": dict(self.status)
                })
                self.health_monitor.log_error(f"Connection failed: {e}", {
                    "channels": self.config.channels,
                    "status": dict(self.status)
                })
                
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
                self.status["run"] = tc08.usb_tc08_run(self.chandle.value, self.config.sample_interval_ms)
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
                
                self.status["stop"] = tc08.usb_tc08_stop(self.chandle.value)
                assert_pico2000_ok(self.status["stop"])
                
                self.streaming = False
                logger.info("TC-08 streaming stopped")
                return True
                
            except Exception as e:
                logger.error(f"Failed to stop streaming: {e}")
                return False
    
    def read_temperatures(self) -> Dict[int, TemperatureReading]:
        """Read temperatures with error logging and failure tracking"""
        if not self.connected:
            raise RuntimeError("Device not connected")
        
        if not self.streaming:
            raise RuntimeError("Device not in streaming mode")
        
        readings = {}
        timestamp = datetime.now()
        failed_channels = 0
        
        try:
            for channel in self.config.channels:
                reading = self._read_single_channel(channel, timestamp)
                readings[channel] = reading
                
                # Count failed channels
                if reading.error:
                    failed_channels += 1
                    # Only log first occurrence and then every 10th to avoid log spam
                    if self.consecutive_read_failures == 0 or self.total_read_failures % 10 == 0:
                        self.health_monitor.log_error(f"Channel {channel} read error: {reading.error}")
            
            # Track failure patterns
            if failed_channels == len(self.config.channels):
                # All channels failed - likely transient EMI or USB issue
                self.consecutive_read_failures += 1
                self.total_read_failures += 1
                self.last_failed_read = timestamp
                
                # Get detailed error diagnostics
                error_info = self.get_last_error()
                
                logger.warning(f"TC-08 missed reading #{self.consecutive_read_failures} "
                             f"(total: {self.total_read_failures}) - all channels unavailable")
                logger.warning(f"TC-08 Error: {error_info['error_description']}")
                
                # Log to health monitor with error diagnostics
                self.health_monitor.log_error(
                    f"All channels failed (#{self.consecutive_read_failures})",
                    {
                        'consecutive_failures': self.consecutive_read_failures,
                        'total_failures': self.total_read_failures,
                        'error_diagnostics': error_info,
                        'channels': self.config.channels
                    }
                )
            else:
                # At least some channels worked - reset consecutive counter
                if self.consecutive_read_failures > 0:
                    logger.info(f"TC-08 recovered after {self.consecutive_read_failures} consecutive failures")
                self.consecutive_read_failures = 0
                self.last_successful_read = timestamp
            
            return readings
            
        except Exception as e:
            self.consecutive_read_failures += 1
            self.total_read_failures += 1
            self.health_monitor.log_error(f"Temperature read failed: {e}", {
                "channels": self.config.channels,
                "streaming": self.streaming,
                "consecutive_failures": self.consecutive_read_failures
            })
            raise
    
    def _read_single_channel(self, channel: int, timestamp: datetime) -> TemperatureReading:
        """Read temperature from a single channel"""
        try:
            # Prepare buffers
            temp_buffer = (ctypes.c_float * self.config.max_buffer_size)()
            times_ms_buffer = (ctypes.c_int32 * self.config.max_buffer_size)()
            overflow = ctypes.c_int16()
            
            # Read channel data
            readings_count = tc08.usb_tc08_get_temp(
                self.chandle.value,
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
            'handle': int(self.chandle.value) if self.connected else None,
            'consecutive_failures': self.consecutive_read_failures,
            'total_failures': self.total_read_failures,
            'last_successful_read': self.last_successful_read,
            'last_failed_read': self.last_failed_read
        }
    
    def needs_reconnection(self, failure_threshold: int = 5) -> bool:
        """Check if device needs reconnection based on consecutive failures
        
        Args:
            failure_threshold: Number of consecutive failures before reconnection needed
        
        Returns:
            True if reconnection should be attempted
        """
        return self.consecutive_read_failures >= failure_threshold
    
    def disconnect(self) -> bool:
        """
        Disconnect from TC-08 device with robust error handling
        Returns: True if successful, False otherwise
        """
        with self._lock:
            if not self.connected:
                return True
            
            # Force stop streaming with error isolation
            if self.streaming:
                try:
                    self.status["stop"] = tc08.usb_tc08_stop(self.chandle.value)
                    # Don't assert here - continue with cleanup even if stop fails
                    self.streaming = False
                    logger.info("TC-08 streaming stopped")
                except Exception as e:
                    logger.warning(f"Error stopping streaming during disconnect: {e}")
                    self.streaming = False  # Force flag reset
            
            # Force close device handle - this must happen regardless of streaming errors
            try:
                if self.chandle.value and self.chandle.value != 0:
                    self.status["close_unit"] = tc08.usb_tc08_close_unit(self.chandle.value)
                    # Log but don't raise - we want cleanup to complete
                    if self.status["close_unit"] != 0:
                        logger.info("TC-08 device handle closed")
                    else:
                        logger.warning(f"TC-08 close returned status: {self.status['close_unit']}")
            except Exception as e:
                logger.error(f"Critical error during TC-08 handle close: {e}")
            finally:
                # Always cleanup state regardless of errors
                self._cleanup_connection()
                logger.info("TC-08 disconnect cleanup completed")
            
            return True
    
    def _cleanup_connection(self):
        """Clean up connection state - force reset everything"""
        self.connected = False
        self.streaming = False
        self.stop_periodic_health_logging()
        self.chandle = ctypes.c_int16(0)  # Explicitly zero the handle
        # Don't clear status - keep for diagnostics
    
    def start_periodic_health_logging(self, interval_minutes: int = 30):
        """Start periodic health monitoring logs"""
        if self._health_logging_active:
            logger.warning("Health logging already active")
            return
        
        self._health_logging_active = True
        
        def _periodic_health_check():
            if self._health_logging_active:
                try:
                    self.health_monitor.log_health_check(self)
                except Exception as e:
                    logger.error(f"Health check failed: {e}")
                
                # Schedule next check
                if self._health_logging_active:
                    self._health_logging_timer = threading.Timer(
                        interval_minutes * 60, _periodic_health_check
                    )
                    self._health_logging_timer.start()
        
        # Start first check
        _periodic_health_check()
        logger.info(f"Started periodic health logging every {interval_minutes} minutes")
    
    def stop_periodic_health_logging(self):
        """Stop periodic health monitoring logs"""
        self._health_logging_active = False
        if self._health_logging_timer:
            self._health_logging_timer.cancel()
            self._health_logging_timer = None
        logger.info("Stopped periodic health logging")
    
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


class TC08HealthMonitor:
    """Diagnostic monitoring for TC-08 USB communication health"""
    
    def __init__(self, log_path: str = "tc08_diagnostics"):
        self.log_path = Path(log_path)
        self.log_path.mkdir(exist_ok=True)
        self.health_log_file = self.log_path / "tc08_health.log"
        self.error_log_file = self.log_path / "tc08_errors.log"
        self.session_start = datetime.now()
        
    def log_health_check(self, controller_instance):
        """Log USB communication health indicators"""
        try:
            timestamp = datetime.now()
            health_data = {
                'timestamp': timestamp.isoformat(),
                'session_duration_hours': (timestamp - self.session_start).total_seconds() / 3600,
                'connected': controller_instance.connected,
                'streaming': controller_instance.streaming,
                'handle_value': int(controller_instance.chandle.value) if controller_instance.chandle else 0,
                'channels': controller_instance.config.channels,
                'sample_interval_ms': controller_instance.config.sample_interval_ms,
            }
            
            # Test basic device communication if connected
            if controller_instance.connected and controller_instance.chandle.value != 0:
                try:
                    # Simple health check - get minimum interval
                    min_interval = tc08.usb_tc08_get_minimum_interval_ms(controller_instance.chandle.value)
                    health_data['min_interval_response'] = min_interval
                    
                    # Detect USB communication degradation
                    if min_interval == 0:
                        health_data['usb_communication'] = 'DEGRADED - device not responding properly'
                        logger.warning("TC-08 USB communication degradation detected (min_interval=0)")
                        
                        # Get detailed error diagnostics
                        error_info = controller_instance.get_last_error()
                        health_data['last_error_diagnostics'] = error_info
                        logger.warning(f"TC-08 Error Diagnostics: {error_info['error_description']}")
                    else:
                        health_data['usb_communication'] = 'OK'
                        health_data['last_error_diagnostics'] = None
                except Exception as e:
                    health_data['usb_communication'] = f'FAILED: {str(e)}'
                    health_data['min_interval_response'] = None
            else:
                health_data['usb_communication'] = 'DISCONNECTED'
                health_data['min_interval_response'] = None
            
            # Append to health log
            with open(self.health_log_file, 'a') as f:
                f.write(f"{json.dumps(health_data)}\n")
                
        except Exception as e:
            self.log_error(f"Health check failed: {e}")
    
    def log_error(self, error_message: str, additional_data: dict = None, relay_states: dict = None):
        """Log TC-08 errors with context and relay state for EMI correlation"""
        try:
            error_data = {
                'timestamp': datetime.now().isoformat(),
                'session_duration_hours': (datetime.now() - self.session_start).total_seconds() / 3600,
                'error_message': str(error_message),
                'additional_data': additional_data or {},
                'relay_states': relay_states  # Track relay switching for EMI correlation
            }
            
            with open(self.error_log_file, 'a') as f:
                f.write(f"{json.dumps(error_data)}\n")
                
            logger.error(f"TC-08 Error logged: {error_message}")
            
        except Exception:
            pass  # Don't let logging errors crash the system
    
    def log_connection_event(self, event_type: str, success: bool, details: dict = None):
        """Log connection/disconnection events"""
        event_data = {
            'timestamp': datetime.now().isoformat(),
            'event_type': event_type,
            'success': success,
            'details': details or {}
        }
        
        with open(self.health_log_file, 'a') as f:
            f.write(f"CONNECTION_EVENT: {json.dumps(event_data)}\n")


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