"""
VescUart - Python library for interfacing with VESC over UART using pyserial

Based on the Arduino VescUart library by SolidGeek
Converted to Python with pyserial support

License: GNU General Public License v3.0
"""

import serial
import struct
import time
from typing import Optional, Tuple
from dataclasses import dataclass
from enum import IntEnum


class CommPacketId(IntEnum):
    """Communication commands for VESC"""
    COMM_FW_VERSION = 0
    COMM_JUMP_TO_BOOTLOADER = 1
    COMM_ERASE_NEW_APP = 2
    COMM_WRITE_NEW_APP_DATA = 3
    COMM_GET_VALUES = 4
    COMM_SET_DUTY = 5
    COMM_SET_CURRENT = 6
    COMM_SET_CURRENT_BRAKE = 7
    COMM_SET_RPM = 8
    COMM_SET_POS = 9
    COMM_SET_HANDBRAKE = 10
    COMM_SET_DETECT = 11
    COMM_SET_SERVO_POS = 12
    COMM_SET_MCCONF = 13
    COMM_GET_MCCONF = 14
    COMM_GET_MCCONF_DEFAULT = 15
    COMM_SET_APPCONF = 16
    COMM_GET_APPCONF = 17
    COMM_GET_APPCONF_DEFAULT = 18
    COMM_SAMPLE_PRINT = 19
    COMM_TERMINAL_CMD = 20
    COMM_PRINT = 21
    COMM_ROTOR_POSITION = 22
    COMM_EXPERIMENT_SAMPLE = 23
    COMM_DETECT_MOTOR_PARAM = 24
    COMM_DETECT_MOTOR_R_L = 25
    COMM_DETECT_MOTOR_FLUX_LINKAGE = 26
    COMM_DETECT_ENCODER = 27
    COMM_DETECT_HALL_FOC = 28
    COMM_REBOOT = 29
    COMM_ALIVE = 30
    COMM_GET_DECODED_PPM = 31
    COMM_GET_DECODED_ADC = 32
    COMM_GET_DECODED_CHUK = 33
    COMM_FORWARD_CAN = 34
    COMM_SET_CHUCK_DATA = 35
    COMM_CUSTOM_APP_DATA = 36


class FaultCode(IntEnum):
    """VESC fault codes"""
    FAULT_CODE_NONE = 0
    FAULT_CODE_OVER_VOLTAGE = 1
    FAULT_CODE_UNDER_VOLTAGE = 2
    FAULT_CODE_DRV = 3
    FAULT_CODE_ABS_OVER_CURRENT = 4
    FAULT_CODE_OVER_TEMP_FET = 5
    FAULT_CODE_OVER_TEMP_MOTOR = 6


@dataclass
class VescData:
    """Telemetry data returned by the VESC"""
    temp_mosfet: float = 0.0
    temp_motor: float = 0.0
    avg_motor_current: float = 0.0
    avg_input_current: float = 0.0
    duty_cycle_now: float = 0.0
    rpm: float = 0.0
    input_voltage: float = 0.0
    amp_hours: float = 0.0
    amp_hours_charged: float = 0.0
    watt_hours: float = 0.0
    watt_hours_charged: float = 0.0
    tachometer: int = 0
    tachometer_abs: int = 0
    fault_code: int = 0
    pid_pos: float = 0.0
    controller_id: int = 0


@dataclass
class NunchuckData:
    """Nunchuck values to send over UART"""
    value_x: int = 127
    value_y: int = 127
    lower_button: bool = False
    upper_button: bool = False


class VescUart:
    """
    Python library for interfacing with VESC motor controllers over UART
    
    Example usage:
        vesc = VescUart("/dev/ttyUSB0")
        if vesc.get_values():
            print(f"RPM: {vesc.data.rpm}")
            print(f"Voltage: {vesc.data.input_voltage}")
    """
    
    def __init__(self, serial_port: str, baudrate: int = 115200, timeout: float = 0.1):
        """
        Initialize VESC UART communication
        
        Args:
            serial_port: Serial port path (e.g., "/dev/ttyUSB0" or "COM3")
            baudrate: Baud rate for serial communication (default: 115200)
            timeout: Serial read timeout in seconds (default: 0.1)
        """
        self.ser = serial.Serial(serial_port, baudrate, timeout=timeout)
        # Keep port and serial settings for reconnects after reboot
        self._serial_port = serial_port
        self._baudrate = baudrate
        self._timeout = timeout
        self.data = VescData()
        self.nunchuck = NunchuckData()
        self.debug = False
        
    def set_debug(self, debug: bool):
        """Enable or disable debug output"""
        self.debug = debug
        
    def close(self):
        """Close the serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            
    def _crc16(self, data: bytes) -> int:
        """
        Calculate CRC16 checksum for VESC protocol
        
        Args:
            data: Bytes to calculate CRC for
            
        Returns:
            CRC16 checksum value
        """
        crc_table = [
            0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
            0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
            0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
            0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
            0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
            0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
            0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
            0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
            0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
            0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
            0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
            0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
            0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
            0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
            0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
            0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
            0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
            0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
            0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
            0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
            0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
            0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
            0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
            0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
            0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
            0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
            0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
            0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
            0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
            0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
            0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
            0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
        ]
        
        crc = 0
        for byte in data:
            crc = crc_table[((crc >> 8) ^ byte) & 0xFF] ^ (crc << 8)
            crc &= 0xFFFF
        return crc
        
    def _pack_payload(self, payload: bytes) -> bytes:
        """
        Pack payload with length, CRC, and framing bytes
        
        Args:
            payload: Payload bytes to pack
            
        Returns:
            Complete message ready to send
        """
        payload_len = len(payload)
        crc = self._crc16(payload)
        
        # Use single-byte length only for values that fit in one byte (0-255).
        # A payload length of 256 requires the two-byte length form.
        if payload_len <= 255:
            message = bytes([2, payload_len]) + payload
        else:
            message = bytes([3, payload_len >> 8, payload_len & 0xFF]) + payload
            
        message += struct.pack('>H', crc)  # Big-endian uint16
        message += bytes([3])  # End byte
        
        if self.debug:
            print(f"Sending: {' '.join(f'{b:02x}' for b in message)}")
            
        return message
        
    def _unpack_payload(self, message: bytes) -> Optional[bytes]:
        """
        Unpack and verify message, extract payload
        
        Args:
            message: Received message
            
        Returns:
            Payload if valid, None otherwise
        """
        if len(message) < 5:
            return None
            
        # Get payload length
        if message[0] == 2:
            payload_len = message[1]
            payload_start = 2
        elif message[0] == 3:
            payload_len = (message[1] << 8) | message[2]
            payload_start = 3
        else:
            if self.debug:
                print("Invalid start byte")
            return None
            
        # Extract payload and CRC
        payload = message[payload_start:payload_start + payload_len]
        crc_received = struct.unpack('>H', message[payload_start + payload_len:payload_start + payload_len + 2])[0]
        crc_calculated = self._crc16(payload)
        
        if self.debug:
            print(f"CRC received: {crc_received:04x}, calculated: {crc_calculated:04x}")
            
        if crc_received == crc_calculated:
            return payload
        else:
            if self.debug:
                print("CRC mismatch!")
            return None
            
    def _receive_message(self, timeout: float = 0.1) -> Optional[bytes]:
        """
        Receive a complete UART message from VESC
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            Payload if successful, None otherwise
        """
        start_time = time.time()
        message = bytearray()
        expected_length = 256
        
        while (time.time() - start_time) < timeout:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                if not byte:
                    continue
                    
                message.extend(byte)
                
                # Determine expected message length
                if len(message) == 2:
                    if message[0] == 2:
                        expected_length = message[1] + 5  # payload + overhead
                    elif message[0] == 3:
                        continue  # Need next byte
                elif len(message) == 3 and message[0] == 3:
                    expected_length = ((message[1] << 8) | message[2]) + 6
                    
                # Check if message is complete
                if len(message) >= expected_length and message[-1] == 3:
                    payload = self._unpack_payload(bytes(message))
                    if payload:
                        return payload
                    else:
                        return None
                        
                # Prevent infinite buffer growth
                if len(message) > 512:
                    if self.debug:
                        print("Message too long, resetting")
                    return None
                    
        if self.debug:
            print("Timeout receiving message")
        return None
        
    def _send_command(self, payload: bytes) -> bool:
        """
        Send a command to VESC
        
        Args:
            payload: Command payload
            
        Returns:
            True if sent successfully
        """
        message = self._pack_payload(payload)
        self.ser.write(message)
        return True
        
    def get_values(self) -> bool:
        """
        Request and receive telemetry values from VESC
        
        Returns:
            True if successful, False otherwise
        """
        if self.debug:
            print("Requesting COMM_GET_VALUES")
            
        # Send command
        command = bytes([CommPacketId.COMM_GET_VALUES])
        self._send_command(command)
        
        # Receive response
        payload = self._receive_message(timeout=0.2)
        
        if payload and len(payload) > 55:
            return self._process_get_values(payload)
        else:
            if self.debug:
                print(f"Invalid response (len={len(payload) if payload else 0})")
            return False
            
    def _process_get_values(self, payload: bytes) -> bool:
        """
        Process COMM_GET_VALUES response
        
        Args:
            payload: Response payload
            
        Returns:
            True if processed successfully
        """
        if payload[0] != CommPacketId.COMM_GET_VALUES:
            return False
            
        ind = 1  # Skip command byte
        
        # Parse according to VESC protocol
        self.data.temp_mosfet = struct.unpack('>h', payload[ind:ind+2])[0] / 10.0
        ind += 2
        
        self.data.temp_motor = struct.unpack('>h', payload[ind:ind+2])[0] / 10.0
        ind += 2
        
        self.data.avg_motor_current = struct.unpack('>i', payload[ind:ind+4])[0] / 100.0
        ind += 4
        
        self.data.avg_input_current = struct.unpack('>i', payload[ind:ind+4])[0] / 100.0
        ind += 4
        
        ind += 8  # Skip avg_id and avg_iq
        
        self.data.duty_cycle_now = struct.unpack('>h', payload[ind:ind+2])[0] / 1000.0
        ind += 2
        
        self.data.rpm = struct.unpack('>i', payload[ind:ind+4])[0]
        ind += 4
        
        self.data.input_voltage = struct.unpack('>h', payload[ind:ind+2])[0] / 10.0
        ind += 2
        
        self.data.amp_hours = struct.unpack('>i', payload[ind:ind+4])[0] / 10000.0
        ind += 4
        
        self.data.amp_hours_charged = struct.unpack('>i', payload[ind:ind+4])[0] / 10000.0
        ind += 4
        
        self.data.watt_hours = struct.unpack('>i', payload[ind:ind+4])[0] / 10000.0
        ind += 4
        
        self.data.watt_hours_charged = struct.unpack('>i', payload[ind:ind+4])[0] / 10000.0
        ind += 4
        
        self.data.tachometer = struct.unpack('>i', payload[ind:ind+4])[0]
        ind += 4
        
        self.data.tachometer_abs = struct.unpack('>i', payload[ind:ind+4])[0]
        ind += 4
        
        self.data.fault_code = payload[ind]
        ind += 1
        
        self.data.pid_pos = struct.unpack('>i', payload[ind:ind+4])[0] / 1000000.0
        ind += 4
        
        self.data.controller_id = payload[ind]
        
        return True
        
    def set_current(self, current: float):
        """
        Set motor current in Amps
        
        Args:
            current: Current in Amps
        """
        payload = bytes([CommPacketId.COMM_SET_CURRENT])
        payload += struct.pack('>i', int(current * 1000))
        self._send_command(payload)
        
    def set_brake_current(self, brake_current: float):
        """
        Set brake current in Amps
        
        Args:
            brake_current: Brake current in Amps
        """
        payload = bytes([CommPacketId.COMM_SET_CURRENT_BRAKE])
        payload += struct.pack('>i', int(brake_current * 1000))
        self._send_command(payload)
        
    def set_rpm(self, rpm: float):
        """
        Set motor RPM (electrical RPM)
        
        Args:
            rpm: RPM value
        """
        payload = bytes([CommPacketId.COMM_SET_RPM])
        payload += struct.pack('>i', int(rpm))
        self._send_command(payload)
        
    def set_duty(self, duty: float):
        """
        Set motor duty cycle
        
        Args:
            duty: Duty cycle (0.0 to 1.0)
        """
        payload = bytes([CommPacketId.COMM_SET_DUTY])
        payload += struct.pack('>i', int(duty * 100000))
        self._send_command(payload)
        
    def set_handbrake_current(self, brake_current: float):
        """
        Set hand brake current in Amps
        
        Args:
            brake_current: Hand brake current in Amps
        """
        payload = bytes([CommPacketId.COMM_SET_HANDBRAKE])
        payload += struct.pack('>i', int(brake_current * 1000))
        self._send_command(payload)
        
    def set_nunchuck_values(self):
        """
        Send nunchuck joystick and button values to VESC
        """
        payload = bytearray([CommPacketId.COMM_SET_CHUCK_DATA])
        payload.append(self.nunchuck.value_x)
        payload.append(self.nunchuck.value_y)
        payload.append(1 if self.nunchuck.lower_button else 0)
        payload.append(1 if self.nunchuck.upper_button else 0)
        
        # Acceleration data (not used, 6 bytes of zeros)
        payload.extend([0, 0, 0, 0, 0, 0])
        
        self._send_command(bytes(payload))
    
    def get_mcconf(self):
        """
        Get motor configuration from VESC
        
        Returns:
            Dictionary with configuration or None if failed
        """
        if self.debug:
            print("Requesting COMM_GET_MCCONF (command 14)")
            
        command = bytes([14])  # COMM_GET_MCCONF (14, not 13)
        self._send_command(command)
        
        payload = self._receive_message(timeout=0.5)
        
        if self.debug:
            if payload:
                print(f"Received payload: {len(payload)} bytes")
                print(f"First 20 bytes: {' '.join(f'{b:02x}' for b in payload[:20])}")
            else:
                print("No payload received")
        
        if not payload:
            if self.debug:
                print("Failed to get mcconf (no payload)")
            return None
            
        if len(payload) < 100:  # Lower threshold for initial testing
            if self.debug:
                print(f"Warning: payload is short ({len(payload)} bytes, expected ~458)")
                print(f"Payload command byte: {payload[0] if len(payload) > 0 else 'N/A'}")
            # Don't fail, try to parse what we have
        
        # Parse the configuration
        config = {}
        
        try:
            # Speed PID at position 330 (confirmed from analysis)
            if len(payload) >= 351:  # Need at least up to position 350
                # VESC uses little-endian float representation on most firmwares
                config['s_pid_kp'] = struct.unpack('<f', payload[330:334])[0]
                config['s_pid_ki'] = struct.unpack('<f', payload[334:338])[0]
                config['s_pid_kd'] = struct.unpack('<f', payload[338:342])[0]
                config['s_pid_kd_filter'] = struct.unpack('<f', payload[342:346])[0]
                config['s_pid_min_erpm'] = struct.unpack('<f', payload[346:350])[0]
                config['s_pid_allow_braking'] = payload[350] != 0
                
                # Store the full payload for modification
                config['_raw_payload'] = payload
                
                if self.debug:
                    print(f"Parsed Kp: {config['s_pid_kp']:.10f}")
                    print(f"Parsed Ki: {config['s_pid_ki']:.10f}")
                    print(f"Parsed Kd: {config['s_pid_kd']:.10f}")
                    print(f"Parsed Kd Filter: {config['s_pid_kd_filter']:.10f}")                
                return config
            else:
                if self.debug:
                    print(f"Payload too short to parse PID values (need 351, got {len(payload)})")
                return None
                
        except Exception as e:
            if self.debug:
                print(f"Error parsing mcconf: {e}")
                import traceback
                traceback.print_exc()
            return None
    
    def set_kp(self, kp_value, apply_reboot: bool = False):
        """
        Set Speed PID Kp value
        
        This function:
        1. Reads current motor configuration
        2. Modifies only the Kp value
        3. Writes the entire config back to VESC
        
        Args:
            kp_value: New Kp value (typically 0.0001 to 0.01)
            
        Returns:
            True if successful, False otherwise
        """
        # First, get current configuration
        config = self.get_mcconf()
        
        if not config:
            if self.debug:
                print("ERROR: Failed to get current configuration")
            return False
        
        current_kp = config.get('s_pid_kp', 0)
        
        # Get the raw payload
        payload = bytearray(config['_raw_payload'])
        
        # Modify Kp at position 330
        # Pack floats as little-endian to match VESC firmware
        kp_bytes = struct.pack('<f', kp_value)
        payload[330:334] = kp_bytes
        
        # Remove the command byte (index 0) from payload since we'll add it
        # The payload starts with command byte 14 (COMM_GET_MCCONF response)
        # We need to send with command byte 13 (COMM_SET_MCCONF)
        config_data = payload[1:]  # Remove response command byte
        
        # Build COMM_SET_MCCONF command
        set_payload = bytearray([13])  # COMM_SET_MCCONF
        set_payload.extend(config_data)
        
        # Send the modified configuration
        self._send_command(bytes(set_payload))
        
        # Wait for VESC to process and respond
        time.sleep(0.2)
        
        # Clear the serial buffer to remove the SET_MCCONF response
        # The VESC sends back a confirmation (command byte 13)
        if self.ser.in_waiting > 0:
            self.ser.read(self.ser.in_waiting)
        
        # Wait a bit more for settings to apply
        time.sleep(0.1)

        # Optionally reboot the device so new config is applied and reconnect
        if apply_reboot:
            ok = self._reboot_device(wait_seconds=1.0)
            if not ok and self.debug:
                print("Warning: reboot requested but reconnection failed")

        # Verify by reading back (only if debug mode)
        if self.debug:
            verify_config = self.get_mcconf()
            if verify_config:
                new_kp = verify_config.get('s_pid_kp', 0)
                if abs(new_kp - kp_value) < 0.000001:
                    print(f"SUCCESS: Kp changed from {current_kp:.6f} to {new_kp:.6f}")
                    return True
                else:
                    print(f"ERROR: Kp mismatch (expected {kp_value:.6f}, got {new_kp:.6f})")
                    return False

        return True

    def _reboot_device(self, wait_seconds: float = 1.0):
        """Send reboot command to VESC so it reloads configuration from flash.

        Note: reboot will disconnect/reconnect USB; caller should handle reconnection.
        """
        try:
            if self.debug:
                print("Sending reboot command to VESC...")
            # send reboot
            self._send_command(bytes([CommPacketId.COMM_REBOOT]))
        except Exception:
            # ignore send errors during reboot
            pass

        # Close serial and attempt to reopen until device responds or timeout
        try:
            try:
                self.ser.close()
            except Exception:
                pass

            # Wait briefly for device to reboot
            time.sleep(wait_seconds)

            # Try reconnecting a few times
            reconnect_attempts = 10
            for i in range(reconnect_attempts):
                try:
                    self.ser = serial.Serial(self._serial_port, self._baudrate, timeout=self._timeout)
                    if self.debug:
                        print(f"Reconnected to {self._serial_port} on attempt {i+1}")
                    return True
                except Exception:
                    time.sleep(0.5)

        except Exception:
            pass

        if self.debug:
            print("Failed to reconnect after reboot")
        return False
        
    def set_ki(self, ki_value, apply_reboot: bool = False):
        """
        Set Speed PID Ki value
        
        Similar to set_kp, but modifies the Ki value at position 334
        
        Args:
            ki_value: New Ki value (typically 0.0001 to 0.01)
            
        Returns:
            True if successful, False otherwise
        """
        # First, get current configuration
        config = self.get_mcconf()
        
        if not config:
            if self.debug:
                print("ERROR: Failed to get current configuration")
            return False
        
        current_ki = config.get('s_pid_ki', 0)
        
        # Get the raw payload
        payload = bytearray(config['_raw_payload'])
        
        # Modify Ki at position 334
        ki_bytes = struct.pack('<f', ki_value)
        payload[334:338] = ki_bytes
        
        # Remove the command byte (index 0) from payload since we'll add it
        config_data = payload[1:]  # Remove response command byte
        
        # Build COMM_SET_MCCONF command
        set_payload = bytearray([13])  # COMM_SET_MCCONF
        set_payload.extend(config_data)
        
        # Send the modified configuration
        self._send_command(bytes(set_payload))
        
        # Wait for VESC to process and respond
        time.sleep(0.2)
        
        # Clear the serial buffer to remove the SET_MCCONF response
        if self.ser.in_waiting > 0:
            self.ser.read(self.ser.in_waiting)
        
        # Wait a bit more for settings to apply
        time.sleep(0.1)

        # Optionally reboot the device so new config is applied and reconnect
        if apply_reboot:
            ok = self._reboot_device(wait_seconds=1.0)
            if not ok and self.debug:
                print("Warning: reboot requested but reconnection failed")

        # Verify by reading back (only if debug mode)
        if self.debug:
            verify_config = self.get_mcconf()
            if verify_config:
                new_ki = verify_config.get('s_pid_ki', 0)
                if abs(new_ki - ki_value) < 0.000001:
                    print(f"SUCCESS: Ki changed from {current_ki:.6f} to {new_ki:.6f}")
                    return True
                else:
                    print(f"ERROR: Ki mismatch (expected {ki_value:.6f}, got {new_ki:.6f})")
                    return False

        return True
    
    def set_kd(self, kd_value, apply_reboot: bool = False):
        """
        Set Speed PID Kd value
        
        Similar to set_kp, but modifies the Kd value at position 338
        
        Args:
            kd_value: New Kd value (typically 0.0001 to 0.01)
            
        Returns:
            True if successful, False otherwise
        """
        # First, get current configuration
        config = self.get_mcconf()
        
        if not config:
            if self.debug:
                print("ERROR: Failed to get current configuration")
            return False
        
        current_kd = config.get('s_pid_kd', 0)
        
        # Get the raw payload
        payload = bytearray(config['_raw_payload'])
        
        # Modify Kd at position 338
        kd_bytes = struct.pack('<f', kd_value)
        payload[338:342] = kd_bytes
        
        # Remove the command byte (index 0) from payload since we'll add it
        config_data = payload[1:]  # Remove response command byte
        
        # Build COMM_SET_MCCONF command
        set_payload = bytearray([13])  # COMM_SET_MCCONF
        set_payload.extend(config_data)
        
        # Send the modified configuration
        self._send_command(bytes(set_payload))
        
        # Wait for VESC to process and respond
        time.sleep(0.2)
        
        # Clear the serial buffer to remove the SET_MCCONF response
        if self.ser.in_waiting > 0:
            self.ser.read(self.ser.in_waiting)
        
        # Wait a bit more for settings to apply
        time.sleep(0.1)

        # Optionally reboot the device so new config is applied and reconnect
        if apply_reboot:
            ok = self._reboot_device(wait_seconds=1.0)
            if not ok and self.debug:
                print("Warning: reboot requested but reconnection failed")

        # Verify by reading back (only if debug mode)
        if self.debug:
            verify_config = self.get_mcconf()
            if verify_config:
                new_kd = verify_config.get('s_pid_kd', 0)
                if abs(new_kd - kd_value) < 0.000001:
                    print(f"SUCCESS: Kd changed from {current_kd:.6f} to {new_kd:.6f}")
                    return True
                else:
                    print(f"ERROR: Kd mismatch (expected {kd_value:.6f}, got {new_kd:.6f})")
                    return False

        return True

    def print_values(self):
        """Print current VESC telemetry values"""
        print(f"Temperature MOSFET: {self.data.temp_mosfet:.1f}°C")
        print(f"Temperature Motor: {self.data.temp_motor:.1f}°C")
        print(f"Avg Motor Current: {self.data.avg_motor_current:.2f}A")
        print(f"Avg Input Current: {self.data.avg_input_current:.2f}A")
        print(f"Duty Cycle: {self.data.duty_cycle_now:.3f}")
        print(f"RPM: {self.data.rpm:.0f}")
        print(f"Input Voltage: {self.data.input_voltage:.1f}V")
        print(f"Amp Hours: {self.data.amp_hours:.3f}Ah")
        print(f"Amp Hours Charged: {self.data.amp_hours_charged:.3f}Ah")
        print(f"Watt Hours: {self.data.watt_hours:.3f}Wh")
        print(f"Watt Hours Charged: {self.data.watt_hours_charged:.3f}Wh")
        print(f"Tachometer: {self.data.tachometer}")
        print(f"Tachometer Abs: {self.data.tachometer_abs}")
        print(f"Fault Code: {self.data.fault_code}")
        print(f"PID Position: {self.data.pid_pos:.6f}")
        print(f"Controller ID: {self.data.controller_id}")


if __name__ == "__main__":
    # Example usage
    print("VescUart Python Library")
    print("Example: vesc = VescUart('/dev/ttyUSB0')")
    # Replace the SERIAL_PORT value below with your system port,
    # e.g. "COM5" on Windows or "/dev/ttyUSB0" on Linux/macOS.
    SERIAL_PORT = "COM5"

    try:
        vesc = VescUart(SERIAL_PORT)  # Connect to VESC

        if vesc.get_values():  # Request telemetry
            print(f"RPM: {vesc.data.rpm}")
            print(f"Voltage: {vesc.data.input_voltage} V")

        # Uncomment the next line to command the motor (use with caution)
        # vesc.set_rpm(3000)

    except Exception as e:
        print(f"Could not open serial port {SERIAL_PORT}: {e}")
    finally:
        try:
            vesc.close()
        except Exception:
            pass