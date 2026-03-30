import serial
import time

def send_command(port="/dev/tty.usbserial-140", baudrate=9600, command="S"):
    # Open the serial connection
    ser = serial.Serial(
            port=port,  # Port name (adjust as necessary)
            baudrate=baudrate,  # Baud rate
            stopbits=serial.STOPBITS_ONE,  # Stop bits (1 stop bit)
            parity=serial.PARITY_NONE,  # No parity
            bytesize=serial.EIGHTBITS,  # 8 data bits
            rtscts=False,  # No hardware flow control
            timeout=10,  # Read timeout in secondspπ
            write_timeout=1,  # Write timeout in seconds
            xonxoff=False  # No software flow control
            )

    time.sleep(0.2)  # small delay for stability

    # Send command
    ser.write(command.encode('utf-8'))
    print(f"Sent: {command}")

    # Read response (if any)
    response = ser.readline().decode('utf-8', errors='ignore').strip()

    if response:
        print("Received:", response)
    else:
        print("No response (timeout).")

    ser.close()

if __name__ == "__main__":
    send_command()