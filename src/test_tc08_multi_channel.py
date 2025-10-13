"""
TC-08 Multi-Channel Streaming Test for Shuttlebox Thermoregulation
Tests K-type thermocouple readings from multiple channels simultaneously
- Channels 1-3: Warm side (inlet, center, buffer)
- Channels 4-6: Cool side (inlet, center, buffer)

Usage:
1. Connect TC-08 to PC via USB
2. Connect K-type thermocouples to desired channels
3. Run this script
4. Monitor real-time temperature readings from all channels

Based on correct TC-08 API usage - each channel read separately
"""

import ctypes
import time
from datetime import datetime
from picosdk.usbtc08 import usbtc08 as tc08
from picosdk.functions import assert_pico2000_ok

class TC08MultiChannelTester:
    def __init__(self, channels=None):
        self.chandle = ctypes.c_int16()
        self.status = {}
        self.typeK = ctypes.c_int8(75)  # K-type thermocouple
        self.channels = channels or [1, 2, 3, 4, 5, 6]  # Default: all 6 shuttlebox channels
        self.channel_names = {
            1: "Warm Inlet",
            2: "Warm Center", 
            3: "Warm Buffer",
            4: "Cool Inlet",
            5: "Cool Center",
            6: "Cool Buffer"
        }
        self.connected = False
        self.units = 0  # Centigrade
        
    def connect(self):
        """Initialize TC-08 connection for multiple channels"""
        try:
            # Open unit
            self.status["open_unit"] = tc08.usb_tc08_open_unit()
            assert_pico2000_ok(self.status["open_unit"])
            self.chandle = self.status["open_unit"]
            
            # Set mains rejection to 50 Hz
            self.status["set_mains"] = tc08.usb_tc08_set_mains(self.chandle, 0)
            assert_pico2000_ok(self.status["set_mains"])
            
            # Configure all requested channels
            for channel in self.channels:
                status_key = f"set_channel_{channel}"
                self.status[status_key] = tc08.usb_tc08_set_channel(self.chandle, channel, self.typeK)
                assert_pico2000_ok(self.status[status_key])
            
            # Get minimum sampling interval
            self.status["get_minimum_interval_ms"] = tc08.usb_tc08_get_minimum_interval_ms(self.chandle)
            assert_pico2000_ok(self.status["get_minimum_interval_ms"])
            
            # Start streaming
            self.status["run"] = tc08.usb_tc08_run(self.chandle, self.status["get_minimum_interval_ms"])
            assert_pico2000_ok(self.status["run"])
            
            self.connected = True
            print(f"✅ TC-08 connected successfully")
            print(f"   Channels: {self.channels}")
            print(f"   Minimum interval: {self.status['get_minimum_interval_ms']} ms")
            
            # Wait for stabilization
            print("⏳ Waiting for sensor stabilization...")
            time.sleep(3)
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to connect TC-08: {e}")
            return False
    
    def read_single_channel(self, channel):
        """Read temperature from a single channel"""
        try:
            # Buffer for single channel readings
            max_readings = 50
            temp_buffer = (ctypes.c_float * max_readings)()
            times_ms_buffer = (ctypes.c_int32 * max_readings)()
            overflow = ctypes.c_int16()
            
            # Get temperature readings for this specific channel
            readings_count = tc08.usb_tc08_get_temp(
                self.chandle, 
                ctypes.byref(temp_buffer),
                ctypes.byref(times_ms_buffer),
                max_readings,
                ctypes.byref(overflow),
                channel,    # Read this channel only
                self.units, # Temperature units (Centigrade)
                1           # Fill missing readings
            )
            
            if readings_count > 0:
                # Get the most recent reading (first in buffer)
                latest_temp = float(temp_buffer[0])
                return {
                    'channel': channel,
                    'temperature': latest_temp,
                    'readings_count': readings_count,
                    'overflow': overflow.value > 0,
                    'timestamp': time.time()
                }
            else:
                return {
                    'channel': channel,
                    'temperature': None,
                    'readings_count': 0,
                    'overflow': False,
                    'timestamp': time.time()
                }
                
        except Exception as e:
            print(f"❌ Error reading channel {channel}: {e}")
            return {
                'channel': channel,
                'temperature': None,
                'readings_count': 0,
                'overflow': False,
                'timestamp': time.time(),
                'error': str(e)
            }
    
    def read_all_channels(self):
        """Read temperatures from all configured channels"""
        if not self.connected:
            print("❌ TC-08 not connected")
            return None
        
        channel_data = {}
        for channel in self.channels:
            data = self.read_single_channel(channel)
            channel_data[channel] = data
        
        return channel_data
    
    def get_cold_junction_temp(self):
        """Get cold junction temperature using single read mode"""
        try:
            # Temporarily stop streaming to get single reading
            self.status["stop"] = tc08.usb_tc08_stop(self.chandle)
            assert_pico2000_ok(self.status["stop"])
            
            # Get single reading including cold junction
            temp_array = (ctypes.c_float * 9)()  # 9 channels (0-8)
            overflow = ctypes.c_int16()
            
            status = tc08.usb_tc08_get_single(
                self.chandle,
                ctypes.byref(temp_array),
                ctypes.byref(overflow),
                self.units
            )
            
            cj_temp = None
            if status == 1:
                cj_temp = float(temp_array[0])  # Channel 0 = cold junction
            
            # Restart streaming
            self.status["run"] = tc08.usb_tc08_run(self.chandle, self.status["get_minimum_interval_ms"])
            assert_pico2000_ok(self.status["run"])
            
            return cj_temp
            
        except Exception as e:
            print(f"❌ Error reading cold junction: {e}")
            # Ensure streaming is restarted
            try:
                self.status["run"] = tc08.usb_tc08_run(self.chandle, self.status["get_minimum_interval_ms"])
                assert_pico2000_ok(self.status["run"])
            except:
                pass
            return None
    
    def print_temperatures(self, channel_data, cj_temp=None):
        """Print formatted temperature readings"""
        if not channel_data:
            print("❌ No temperature data")
            return
        
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"\n[{timestamp}] Multi-Channel Temperature Readings:")
        
        if cj_temp is not None:
            print(f"   Cold Junction: {cj_temp:6.2f}°C")
        
        # Warm side channels
        print("   WARM SIDE:")
        for ch in [1, 2, 3]:
            if ch in channel_data and ch in self.channels:
                data = channel_data[ch]
                name = self.channel_names.get(ch, f"Channel {ch}")
                if data['temperature'] is not None:
                    status = "⚠️" if data['overflow'] else "✅"
                    print(f"     Ch{ch} {name:12s}: {data['temperature']:6.2f}°C {status} ({data['readings_count']} readings)")
                else:
                    print(f"     Ch{ch} {name:12s}: ❌ No data")
        
        # Cool side channels
        print("   COOL SIDE:")
        for ch in [4, 5, 6]:
            if ch in channel_data and ch in self.channels:
                data = channel_data[ch]
                name = self.channel_names.get(ch, f"Channel {ch}")
                if data['temperature'] is not None:
                    status = "⚠️" if data['overflow'] else "✅"
                    print(f"     Ch{ch} {name:12s}: {data['temperature']:6.2f}°C {status} ({data['readings_count']} readings)")
                else:
                    print(f"     Ch{ch} {name:12s}: ❌ No data")
        
        # Calculate temperature differences
        warm_temps = [channel_data[ch]['temperature'] for ch in [1, 2, 3] 
                     if ch in channel_data and channel_data[ch]['temperature'] is not None]
        cool_temps = [channel_data[ch]['temperature'] for ch in [4, 5, 6] 
                     if ch in channel_data and channel_data[ch]['temperature'] is not None]
        
        if warm_temps and cool_temps:
            avg_warm = sum(warm_temps) / len(warm_temps)
            avg_cool = sum(cool_temps) / len(cool_temps)
            diff = avg_warm - avg_cool
            print(f"   TEMPERATURE DIFFERENCE (Warm-Cool): {diff:+6.2f}°C")
    
    def stream_temperatures(self, duration_seconds=None, interval_seconds=2, include_cj=False):
        """Stream temperature readings continuously from all channels"""
        if not self.connected:
            print("❌ TC-08 not connected")
            return
        
        print(f"\n🌡️ Starting multi-channel streaming...")
        print(f"   Channels: {self.channels}")
        if duration_seconds:
            print(f"   Duration: {duration_seconds} seconds")
        else:
            print(f"   Duration: Continuous (Press Ctrl+C to stop)")
        print(f"   Interval: {interval_seconds} seconds")
        print(f"   Cold Junction: {'Included' if include_cj else 'Disabled (faster)'}")
        
        start_time = time.time()
        reading_count = 0
        
        try:
            while True:
                # Read all channel temperatures
                channel_data = self.read_all_channels()
                
                # Optionally read cold junction (slower due to mode switching)
                cj_temp = self.get_cold_junction_temp() if include_cj else None
                
                if channel_data:
                    self.print_temperatures(channel_data, cj_temp)
                    reading_count += 1
                
                # Check duration limit
                if duration_seconds and (time.time() - start_time) >= duration_seconds:
                    break
                
                time.sleep(interval_seconds)
                
        except KeyboardInterrupt:
            print(f"\n⏹️ Streaming stopped by user")
        
        print(f"📊 Total readings: {reading_count}")
    
    def disconnect(self):
        """Close TC-08 connection"""
        if not self.connected:
            return
        
        try:
            # Stop streaming
            self.status["stop"] = tc08.usb_tc08_stop(self.chandle)
            assert_pico2000_ok(self.status["stop"])
            
            # Close unit
            self.status["close_unit"] = tc08.usb_tc08_close_unit(self.chandle)
            assert_pico2000_ok(self.status["close_unit"])
            
            self.connected = False
            print("✅ TC-08 disconnected successfully")
            
        except Exception as e:
            print(f"❌ Error disconnecting TC-08: {e}")


def main():
    """Main test routine"""
    print("="*70)
    print("TC-08 Multi-Channel Streaming Test - Shuttlebox Thermoregulation")
    print("="*70)
    
    # Ask user which channels to test
    print("Available channels:")
    print("  1-3: Warm side (inlet, center, buffer)")
    print("  4-6: Cool side (inlet, center, buffer)")
    print("  Or specify custom channels (e.g., 1,3,5)")
    
    channels_input = input("Enter channels to test (default: 1,2,3,4,5,6): ").strip()
    
    if channels_input:
        try:
            channels = [int(x.strip()) for x in channels_input.split(',')]
            channels = [ch for ch in channels if 1 <= ch <= 8]  # Validate range
        except ValueError:
            channels = [1, 2, 3, 4, 5, 6]
    else:
        channels = [1, 2, 3, 4, 5, 6]
    
    tester = TC08MultiChannelTester(channels=channels)
    
    try:
        # Connect to TC-08
        if not tester.connect():
            return
        
        # Test single reading from all channels
        print(f"\n📍 Single reading from all channels:")
        channel_data = tester.read_all_channels()
        tester.print_temperatures(channel_data)
        
        # Ask user for streaming test
        print(f"\n" + "="*70)
        choice = input("Start continuous streaming? (y/N): ").strip().lower()
        
        if choice in ['y', 'yes']:
            try:
                interval = float(input("Sampling interval in seconds (default 2.0): ") or "2.0")
            except ValueError:
                interval = 2.0
            
            # Ask about cold junction
            cj_choice = input("Include cold junction readings? (slower) (y/N): ").strip().lower()
            include_cj = cj_choice in ['y', 'yes']
            
            tester.stream_temperatures(interval_seconds=interval, include_cj=include_cj)
    
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    
    finally:
        tester.disconnect()
        print("\n✅ Test completed")


if __name__ == "__main__":
    main()