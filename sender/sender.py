from PIL import Image
import numpy as np
import socket
import struct

def send_bitmap(bitmap, width, height, ip, port):
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Convert bitmap to bytes
    bitmap_bytes = bytearray(bitmap)
    
    # Packet metadata
    header_format = 'HHH'  # Width, Height, Packet Number
    packet_size = 1024  # Arbitrary packet size
    total_packets = (len(bitmap_bytes) + packet_size - 1) // packet_size
    
    for packet_num in range(total_packets):
        start = packet_num * packet_size
        end = start + packet_size
        chunk = bitmap_bytes[start:end]
        
        # Construct packet with header and payload
        header = struct.pack(header_format, width, height, packet_num)
        packet = header + chunk
        
        # Send packet
        sock.sendto(packet, (ip, port))

    # Send end signal
    end_signal = struct.pack(header_format, 0, 0, 65535)
    sock.sendto(end_signal, (ip, port))

    sock.close()

def read(image_path):
    return Image.open(image_path)

def threshold(img, threshold=None):
    
    img = img.convert('L')

    # Convert to numpy array
    img_array = np.array(img)

    if threshold is None:
        max_value = img_array.max()
        min_value = img_array.min()
        threshold = min_value + (max_value - min_value) / 2
    
    # Threshold to binary black and white
    img_array[img_array < threshold] = 0
    img_array[img_array >= threshold] = 255
    
    return img_array

def pad(image_array, target_width=128, target_height=64):
    padded_image = np.zeros((target_height, target_width), dtype=np.uint8)
    print(image_array.shape)
    height, width = image_array.shape
    start_x = (target_width - width) // 2
    start_y = (target_height - height) // 2
    padded_image[start_y:start_y + height, start_x:start_x + width] = image_array
    return padded_image

def flatten(image_array):
    return image_array.flatten()

"""
def generate_striped_bitmap(width, height):
    # Number of bytes per row
    bytes_per_row = width // 8
    
    bitmap = bytearray()
    
    for row in range(height):
        if row % 2 == 0:
            # Row of all ones
            bitmap.extend([0xFF] * bytes_per_row)
        else:
            # Row of all zeros
            bitmap.extend([0x00] * bytes_per_row)
    
    return bitmap
"""
    
# Example usage
width = 128
height = 64
# bitmap = generate_striped_bitmap(width, height)

image_path = '/home/daniel/Git/Desktop-Companion-Robot/sender/frames/frame_0.png'
image = read(image_path)
bwimage = threshold(image)
padded = pad(bwimage)
bitmap = flatten(padded)

ip = '192.168.1.8'
port = 12345

"""
# Printing the bitmap for visualization (optional)
for i in range(height):
    row = bitmap[i * (width // 8):(i + 1) * (width // 8)]
    print(' '.join(f'{byte:08b}' for byte in row))
"""

send_bitmap(bitmap, width, height, ip, port)
