import serial
from pynput.mouse import Controller, Button
import time

# Initialize serial connection
ser = serial.Serial('COM3', 115200)  # Adjust 'COM3' to your actual port

# Initialize mouse controller
mouse = Controller()

# Initialize variables to track previous mouse position and button state
prev_x, prev_y = mouse.position
prev_button_state = 0
button_pressed = False
last_click_time = time.time()

# Debounce threshold (in seconds)
debounce_threshold = 0.1

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()

            if line.startswith("NX:"):
                data = line.split(",")
                try:
                    new_x = int(data[0].split(":")[1])
                    new_y = int(data[1].split(":")[1])
                    button_state = int(data[2].split(":")[1])
                except (IndexError, ValueError) as e:
                    print(f"Error parsing line: {line}, Error: {e}")
                    continue

                # Only update mouse position if it has changed significantly
                if abs(new_x - prev_x) > 1 or abs(new_y - prev_y) > 1:
                    prev_x, prev_y = new_x, new_y
                    mouse.position = (new_x, new_y)

                # Handle mouse click based on button state with debounce
                current_time = time.time()
                if button_state == 1 and not button_pressed:
                    if current_time - last_click_time > debounce_threshold:
                        mouse.press(Button.left)
                        button_pressed = True
                        last_click_time = current_time
                elif button_state == 0 and button_pressed:
                    mouse.release(Button.left)
                    button_pressed = False

except KeyboardInterrupt:
    print("Exiting program")
finally:
    ser.close()
