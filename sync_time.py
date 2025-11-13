# Script Python untuk menyinkronkan waktu dari PC ke ESP32 melalui Serial
# Penggunaan: python sync_time.py <PORT> <HH> <MM> <SS> <DD> <MM> <YY>
# Contoh: python sync_time.py COM5 08 41 46 12 11 25

import sys
import serial
import time
import datetime

# --- Parameter (default to current time) ---
if len(sys.argv) < 2:
    print("Error: Argumen PORT tidak ditemukan.")
    print("Penggunaan: python sync_time.py <PORT> <HH> <MM> <SS> <DD> <MM> <YY>")
    sys.exit(1)

port = sys.argv[1]
try:
    if len(sys.argv) == 8:
        # Gunakan waktu dari argumen
        H, M, S = int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
        D, Mo, Y = int(sys.argv[5]), int(sys.argv[6]), int(sys.argv[7])
        dt = datetime.datetime(2000 + Y, Mo, D, H, M, S)
    else:
        # Gunakan waktu PC saat ini (dengan Y sebagai 2 digit)
        now = datetime.datetime.now()
        H, M, S = now.hour, now.minute, now.second
        D, Mo, Y = now.day, now.month, now.year % 100
        dt = now

    time_command = f"SETTIME {H} {M} {S} {D} {Mo} {Y}"
    print(f"-> Perintah: {time_command}")
    print(f"-> Waktu yang disetel: {dt.strftime('%H:%M:%S %d/%m/%Y')}")
    
except ValueError as e:
    print(f"Error pada format waktu: {e}")
    sys.exit(1)

# --- Serial Communication ---
try:
    print(f"Mencoba menyambung ke port {port}...")
    ser = serial.Serial(port, 115200, timeout=5)
    time.sleep(2) # Tunggu agar port serial terbuka
    
    # Kirim perintah
    ser.write(f"{time_command}\n".encode('utf-8'))
    print("Perintah SETTIME terkirim. Menunggu respon...")
    time.sleep(1) 
    
    # Baca respon (opsional)
    response = ser.read_until(b'\n').decode('utf-8').strip()
    if "Adjusted" in response or "Adjusted" in ser.read_until(b'\n').decode('utf-8').strip():
        print(f"✅ Sinkronisasi SUKSES. Periksa Serial Monitor untuk konfirmasi RTC.")
    else:
        print("⚠️ Sinkronisasi mungkin GAGAL. Cek apakah perangkat sudah booting penuh.")
        
    ser.close()

except serial.SerialException as e:
    print(f"ERROR: Tidak dapat menyambung ke port {port}. Cek kabel atau port COM yang benar.")
    print(f"Detail: {e}")
    sys.exit(1)

