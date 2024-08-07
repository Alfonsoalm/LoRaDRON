import serial
import tkinter as tk
from tkinter import ttk

# Configura el puerto COM y la velocidad de baudios
puerto_com = 'COM20'  # Reemplaza con el puerto correcto
velocidad_baudios = 9600  # Asegúrate de que coincida con la configuración del Arduino

ser = None  # Inicializa ser a None globalmente

# Almacena los últimos valores recibidos
last_pitch = last_roll = last_esc1 = last_esc2 = last_esc3 = last_esc4 = 0.0
last_yaw_angle_target = last_pitch_angle_target = last_roll_angle_target = last_throttle_target = 0.0

def parse_data(line):
    # Inicializa los valores en 0.0
    pitch = roll = esc1 = esc2 = esc3 = esc4 = 0.0
    yaw_angle_target = pitch_angle_target = roll_angle_target = throttle_target = 0.0

    # Divide la línea en componentes individuales
    components = line.split(',')

    # Itera sobre cada componente para encontrar y asignar los valores
    for component in components:
        if 'PITCH' in component:
            pitch = float(component.split(':')[1].strip())
        elif 'ROLL' in component:
            roll = float(component.split(':')[1].strip())
        elif 'ESC1' in component:
            esc1 = float(component.split(':')[1].strip())
        elif 'ESC2' in component:
            esc2 = float(component.split(':')[1].strip())
        elif 'ESC3' in component:
            esc3 = float(component.split(':')[1].strip())
        elif 'ESC4' in component:
            esc4 = float(component.split(':')[1].strip())
        elif 'yaw_angle_target' in component:
            yaw_angle_target = float(component.split(':')[1].strip())
        elif 'pitch_angle_target' in component:
            pitch_angle_target = float(component.split(':')[1].strip())
        elif 'roll_angle_target' in component:
            roll_angle_target = float(component.split(':')[1].strip())
        elif 'throttle_target' in component:
            throttle_target = float(component.split(':')[1].strip())

    return pitch, roll, esc1, esc2, esc3, esc4, yaw_angle_target, pitch_angle_target, roll_angle_target, throttle_target

def read_serial_data():
    global ser, last_pitch, last_roll, last_esc1, last_esc2, last_esc3, last_esc4
    global last_yaw_angle_target, last_pitch_angle_target, last_roll_angle_target, last_throttle_target

    if ser and ser.is_open:
        # Lee una línea completa del puerto serial
        linea = ser.readline().decode('utf-8').strip()

        if linea:
            # Extrae los datos
            try:
                pitch, roll, esc1, esc2, esc3, esc4, yaw_angle_target, pitch_angle_target, roll_angle_target, throttle_target = parse_data(linea)
                # Almacena los últimos valores válidos
                last_pitch, last_roll = pitch, roll
                last_esc1, last_esc2, last_esc3, last_esc4 = esc1, esc2, esc3, esc4
                last_yaw_angle_target, last_pitch_angle_target, last_roll_angle_target, last_throttle_target = yaw_angle_target, pitch_angle_target, roll_angle_target, throttle_target

            except (ValueError, IndexError) as e:
                print(f"Error en la conversión de datos: {linea} ({e})")

        # Actualiza las etiquetas con los últimos valores
        pitch_label.config(text=f"Pitch: {last_pitch:.2f}")
        roll_label.config(text=f"Roll: {last_roll:.2f}")
        esc1_label.config(text=f"ESC1: {last_esc1:.2f}")
        esc2_label.config(text=f"ESC2: {last_esc2:.2f}")
        esc3_label.config(text=f"ESC3: {last_esc3:.2f}")
        esc4_label.config(text=f"ESC4: {last_esc4:.2f}")
        yaw_target_label.config(text=f"Yaw Target: {last_yaw_angle_target:.2f}")
        pitch_target_label.config(text=f"Pitch Target: {last_pitch_angle_target:.2f}")
        roll_target_label.config(text=f"Roll Target: {last_roll_angle_target:.2f}")
        throttle_target_label.config(text=f"Throttle Target: {last_throttle_target:.2f}")

    # Programa la próxima lectura
    root.after(100, read_serial_data)

def start_serial():
    global ser  # Utiliza la variable global ser
    try:
        # Abre el puerto serial
        ser = serial.Serial(puerto_com, velocidad_baudios, timeout=1)
        print(f"Puerto {ser.portstr} abierto")
        read_serial_data()  # Inicia la lectura de datos

    except serial.SerialException as e:
        print(f"Error al abrir el puerto: {e}")
    except Exception as e:
        print(f"Ocurrió un error: {e}")

def on_closing():
    global ser
    if ser:
        ser.close()
    root.destroy()

# Configuración de la interfaz de usuario
root = tk.Tk()
root.title("Monitor de Datos en Tiempo Real")

frame = ttk.Frame(root, padding="10")
frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

pitch_label = ttk.Label(frame, text="Pitch: 0.00")
pitch_label.grid(row=0, column=0, sticky=(tk.W))
roll_label = ttk.Label(frame, text="Roll: 0.00")
roll_label.grid(row=1, column=0, sticky=(tk.W))
esc1_label = ttk.Label(frame, text="ESC1: 0.00")
esc1_label.grid(row=2, column=0, sticky=(tk.W))
esc2_label = ttk.Label(frame, text="ESC2: 0.00")
esc2_label.grid(row=3, column=0, sticky=(tk.W))
esc3_label = ttk.Label(frame, text="ESC3: 0.00")
esc3_label.grid(row=4, column=0, sticky=(tk.W))
esc4_label = ttk.Label(frame, text="ESC4: 0.00")
esc4_label.grid(row=5, column=0, sticky=(tk.W))
yaw_target_label = ttk.Label(frame, text="Yaw Target: 0.00")
yaw_target_label.grid(row=6, column=0, sticky=(tk.W))
pitch_target_label = ttk.Label(frame, text="Pitch Target: 0.00")
pitch_target_label.grid(row=7, column=0, sticky=(tk.W))
roll_target_label = ttk.Label(frame, text="Roll Target: 0.00")
roll_target_label.grid(row=8, column=0, sticky=(tk.W))
throttle_target_label = ttk.Label(frame, text="Throttle Target: 0.00")
throttle_target_label.grid(row=9, column=0, sticky=(tk.W))

# Inicia la lectura del puerto serial
start_serial()

# Maneja el evento de cierre de la ventana
root.protocol("WM_DELETE_WINDOW", on_closing)

# Ejecuta la interfaz
root.mainloop()
