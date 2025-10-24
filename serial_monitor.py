"""
Simple Serial Monitor - Print all data from Teensy to terminal
Usage: python serial_monitor.py COM6 [--interval 50] [--duration 0]
"""

import serial
import sys
import time
import argparse

def monitor_serial(port, baudrate=115200, interval_ms=50, duration_sec=0):
    """
    Monitor serial port and print everything to terminal

    Args:
        port: Serial port (e.g., 'COM6')
        baudrate: Baud rate (default: 115200)
        interval_ms: Sampling interval in milliseconds
        duration_sec: Duration in seconds (0 = continuous)
    """
    try:
        print(f"Connecting to {port} at {baudrate} baud...")
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        print(f"Connected!")

        # Read any initial messages
        time.sleep(0.5)
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"[INIT] {line}")

        # Send START command
        command = f"START,{interval_ms},{duration_sec}"
        print(f"\nSending command: {command}")
        ser.write(f"{command}\n".encode('utf-8'))
        time.sleep(0.5)

        # Read response
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"[RESPONSE] {line}")

        print("\nPress Ctrl+C to stop and exit.\n")
        print("=" * 80)

        line_count = 0

        while True:
            if ser.in_waiting > 0:
                try:
                    # Read line and decode
                    line = ser.readline().decode('utf-8', errors='ignore').strip()

                    if line:
                        line_count += 1
                        # Print with line number
                        print(f"{line_count:6d} | {line}")

                except UnicodeDecodeError:
                    print("[ERROR] Failed to decode line")
                except Exception as e:
                    print(f"[ERROR] {e}")

            time.sleep(0.001)  # Small delay to prevent CPU spinning

    except serial.SerialException as e:
        print(f"[ERROR] Serial error: {e}")
        print("\nPossible causes:")
        print("  1. Port is being used by another program (Arduino IDE, PlatformIO)")
        print("  2. Wrong port name")
        print("  3. Teensy not connected")
        return 1

    except KeyboardInterrupt:
        print("\n" + "=" * 80)
        print(f"\nTotal lines received: {line_count}")
        print("Disconnected.")
        return 0

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

def list_ports():
    """List all available serial ports"""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()

    if not ports:
        print("No serial ports found!")
        return

    print("Available serial ports:")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port.device} - {port.description}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Serial monitor for Teensy dual sensor system',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # List available ports
  python serial_monitor.py --list

  # Monitor with default settings (50ms interval, continuous)
  python serial_monitor.py COM6

  # Monitor with custom interval
  python serial_monitor.py COM6 --interval 20

  # Monitor for 60 seconds
  python serial_monitor.py COM6 --interval 50 --duration 60
        """
    )

    parser.add_argument('port', nargs='?', type=str,
                        help='Serial port (e.g., COM6)')
    parser.add_argument('--interval', '-i', type=int, default=50,
                        help='Sampling interval in milliseconds (default: 50)')
    parser.add_argument('--duration', '-d', type=int, default=0,
                        help='Duration in seconds (0 = continuous, default: 0)')
    parser.add_argument('--list', '-l', action='store_true',
                        help='List available serial ports')

    args = parser.parse_args()

    # List ports if requested
    if args.list:
        list_ports()
        sys.exit(0)

    # Check if port is specified
    if not args.port:
        print("Error: Serial port is required")
        print("\nUsage: python serial_monitor.py <PORT>")
        print("       python serial_monitor.py --list")
        print("\nExample: python serial_monitor.py COM6")
        print()
        list_ports()
        sys.exit(1)

    sys.exit(monitor_serial(args.port, interval_ms=args.interval, duration_sec=args.duration))
