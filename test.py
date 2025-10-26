import socket
import struct

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 12346))  # Listen on port 12346

print("Listening for UDP packets from MCU on port 12346...")
print("Press Ctrl+C to stop")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        print(f"Received {len(data)} bytes from {addr}")
        
        # Parse PS5Data structure (31 bytes)
        if len(data) >= 31:
            # Unpack the data structure
            hat_values = struct.unpack('4B', data[0:4])
            triggers = struct.unpack('2B', data[4:6])
            seq_num = data[6]
            buttons = struct.unpack('3B', data[7:10])
            gyro = struct.unpack('3h', data[15:21])  # 3 int16 values
            accel = struct.unpack('3h', data[21:27])  # 3 int16 values
            timestamp = struct.unpack('I', data[27:31])[0]  # uint32
            
            print(f"  Hat: {hat_values}")
            print(f"  Triggers: {triggers}")
            print(f"  Sequence: {seq_num}")
            print(f"  Buttons: {buttons}")
            print(f"  Gyro: {gyro}")
            print(f"  Accel: {accel}")
            print(f"  Timestamp: {timestamp}")
            print("---")
        
except KeyboardInterrupt:
    print("\nStopped listening")
    sock.close()