import serial

def read_serial_data(port='COM16', baudrate=9600, timeout=1):
    ser = None  # Initialize ser to None
    try:
        # Abre el puerto serial
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"Puerto {ser.portstr} abierto")

        while True:
            if ser.in_waiting > 0:
                # Lee una línea del puerto serial
                line = ser.readline().decode('utf-8').strip()
                print(line)
            else:
                pass  # O puedes poner un pequeño delay con time.sleep() si es necesario

    except serial.SerialException as e:
        print(f"Error al abrir el puerto: {e}")
    except Exception as e:
        print(f"Ocurrió un error: {e}")
    finally:
        if ser:  # Check if ser is not None
            ser.close()

if __name__ == "__main__":
    read_serial_data()
