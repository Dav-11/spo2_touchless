import serial.tools.list_ports
import serial

ports = serial.tools.list_ports.comports()

for port in ports:
    print(f"{port.device}: {port.description}")

# Open connection
ser = serial.Serial('/dev/tty.usbserial-110', 115200, timeout=2)

while True:
    # Read response
    response = ser.readline()
    print(response.decode('utf-8').strip())

    # Read all available
    if ser.in_waiting > 0:
        available = ser.read(ser.in_waiting)
        print(available.decode('utf-8').strip())

# Close connection
ser.close()