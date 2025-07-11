import serial
import time

# Configure your port and baud rate
port = '/dev/cu.usbserial-0001'
baud = 115200

try:
    ser = serial.Serial(port, baud, timeout=5)  # Increased timeout
    time.sleep(2)  # Let ESP32 boot

    print("Connected. Sending export command...")
    ser.write(b'e')  # Send export command

    with open('data.csv', 'w') as f:
        print("Capturing data to data.csv...")

        line_count = 0
        last_data_time = time.time()
        no_data_timeout = 10  # Wait 10 seconds after last data before giving up

        while True:
            line = ser.readline().decode('utf-8', errors='ignore')

            if line.strip():  # If we got actual data (not just whitespace)
                f.write(line)
                f.flush()  # Ensure it's written immediately
                line_count += 1
                last_data_time = time.time()  # Reset the timer

                # Show progress every 1000 lines
                if line_count % 1000 == 0:
                    print(f"Captured {line_count} lines...")

                # Check if this looks like the end of data
                if "END" in line or "DONE" in line or "COMPLETE" in line:
                    print("Found end marker in data")
                    break

            else:
                # No data received, check if we've waited too long
                if time.time() - last_data_time > no_data_timeout:
                    print(
                        f"No data received for {no_data_timeout} seconds. Assuming complete.")
                    break

        print(f"Capture complete! Total lines: {line_count}")

except serial.SerialException as e:
    print(f"Serial port error: {e}")
except KeyboardInterrupt:
    print("\nCapture interrupted by user")
except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")

print("Done! Check data.csv for all data")
