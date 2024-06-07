from PIL import Image

def extract_frames(sprite_sheet_path, frame_width, frame_height, rows, columns):
    # Open the sprite sheet
    sprite_sheet = Image.open(sprite_sheet_path)
    frames = []
    
    for row in range(rows):
        for col in range(columns):
            # Calculate the position of each frame
            left = col * frame_width
            upper = row * frame_height
            right = left + frame_width
            lower = upper + frame_height
            
            # Extract the frame
            frame = sprite_sheet.crop((left, upper, right, lower))
            frames.append(frame)
    
    return frames

def save_frames(frames, output_folder):
    for i, frame in enumerate(frames):
        frame.save(f"{output_folder}/frame_{i}.png")

def save_frames_as_bmp(frames, output_folder):
    for i, frame in enumerate(frames):
        frame.save(f"{output_folder}/frame_{i}.bmp", format='BMP')


# Example usage
sprite_sheet_path = "/home/daniel/Downloads/cat.png"  # Replace with your sprite sheet path
frame_width = 32   # Replace with your frame width
frame_height = 32  # Replace with your frame height
rows = 10          # Replace with number of rows in your sprite sheet
columns = 8        # Replace with number of columns in your sprite sheet

frames = extract_frames(sprite_sheet_path, frame_width, frame_height, rows, columns)
save_frames_as_bmp(frames, "/home/daniel/Git/Desktop-Companion-Robot/sender/frames_bmp")  # Replace "output_frames" with your desired output folder
