#!/usr/bin/env python3
import serial
import time
from datetime import datetime
import sys

# Change PORT if necessary
PORT = "COM5" 
BAUD_RATE = 115200
WAIT_TIME = 8

def sync_time():
    now = datetime.now()
    time_str = now.strftime("%H:%M:%S")
    date_str = now.strftime("%y:%m:%d")
    
    print(f"\n=== ArsLed V601 Manual Time Sync ===")
    print(f"Port: {PORT}")
    print(f"Time: {time_str}")
    print(f"Date: {date_str}")
    print(f"\nWaiting {WAIT_TIME} seconds for ESP32 stability...")
    
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=2)
        time.sleep(WAIT_TIME)
        
        print("\n[1/2] Sending SETTIME command...")
        cmd_time = f"SETTIME={time_str}\n"
        ser.write(cmd_time.encode('utf-8'))
        ser.flush()
        time.sleep(1)
        
        if ser.in_waiting:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"Response: {response.strip()}")
        
        print("\n[2/2] Sending SETDATE command...")
        cmd_date = f"SETDATE={date_str}\n"
        ser.write(cmd_date.encode('utf-8'))
        ser.flush()
        time.sleep(1)
        
        if ser.in_waiting:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"Response: {response.strip()}")
        
        ser.close()
        print("\n✓ Time synchronization completed!")
        
    except serial.SerialException as e:
        print(f"\n✗ ERROR: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        PORT = sys.argv[1] # Allow port override if passed as argument
        
    sync_time()
