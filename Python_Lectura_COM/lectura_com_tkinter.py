import serial
import tkinter as tk
from tkinter import ttk

# Conectarse por bluetooth
# btpair -b 12:29:4B:0F:25:0A -P1234
# btcom -r -b 12:29:4B:0F:25:0A -s1101
# btcom -c -b 12:29:4B:0F:25:0A -s1101

# Configura el puerto COM y la velocidad de baudios
puerto_com = 'COM21'  # Reemplaza con el puerto correcto
velocidad_baudios = 9600  # Asegúrate de que coincida con la configuración del Arduino

ser = None  # Inicializa ser a None globalmente

# Almacena los últimos valores recibidos
last_pitch = last_roll = last_esc1 = last_esc2 = last_esc3 = last_esc4 = 0.0
last_yaw_angle_target = last_pitch_angle_target = last_roll_angle_target = last_throttle_target = 0.0
last_pitch_angle_output = last_roll_angle_output = 0.0

def parse_data(line):
    print(f"Línea recibida: {line}")  # Imprimir la línea completa recibida para depuración

    # Inicializa los valores en 0.0
    data_mapping = {
        'PITCH': 0.0,
        'ROLL': 0.0,
        'YAW_target': 0.0,
        'PITCH_target': 0.0,
        'ROLL_target': 0.0,
        'THROTTLE_target': 0.0,
        'PITCH_output': 0.0,
        'ROLL_output': 0.0,
        'ESC1': 0.0,
        'ESC2': 0.0,
        'ESC3': 0.0,
        'ESC4': 0.0,
    }

    # Dividir la línea en pares clave-valor usando las comas como delimitador
    pairs = line.split(',')
    
    for pair in pairs:
        if ':' in pair:
            key, value_str = pair.split(':', 1)
            key = key.strip()
            value_str = value_str.strip()
            
            # Convertir el valor a float y actualizar el diccionario
            if key in data_mapping:
                try:
                    data_mapping[key] = float(value_str)
                except ValueError:
                    print(f"Error al convertir el valor para {key} en la línea: {line}")

    # Asignación de los valores a variables individuales
    pitch = data_mapping['PITCH']
    roll = data_mapping['ROLL']
    
    yaw_angle_target = data_mapping['YAW_target']
    pitch_angle_target = data_mapping['PITCH_target']
    roll_angle_target = data_mapping['ROLL_target']
    throttle_target = data_mapping['THROTTLE_target']   

    pitch_angle_output = data_mapping['PITCH_output']
    roll_angle_output = data_mapping['ROLL_output']

    esc1 = data_mapping['ESC1']
    esc2 = data_mapping['ESC2']
    esc3 = data_mapping['ESC3']
    esc4 = data_mapping['ESC4']

    print(f"Valores extraídos -> Pitch: {pitch}, Roll: {roll}, ESC1: {esc1}, ESC2: {esc2}, ESC3: {esc3}, ESC4: {esc4}, Yaw Target: {yaw_angle_target}, Pitch Target: {pitch_angle_target}, Roll Target: {roll_angle_target}, Throttle Target: {throttle_target}, Pitch Output: {pitch_angle_output}, Roll Output: {roll_angle_output}")

    return pitch, roll, esc1, esc2, esc3, esc4, yaw_angle_target, pitch_angle_target, roll_angle_target, throttle_target, pitch_angle_output, roll_angle_output

def read_serial_data():
    global ser, last_pitch, last_roll, last_esc1, last_esc2, last_esc3, last_esc4
    global last_yaw_angle_target, last_pitch_angle_target, last_roll_angle_target, last_throttle_target
    global last_pitch_angle_output, last_roll_angle_output

    if ser and ser.is_open:
        # Lee una línea completa del puerto serial
        try:
            linea = ser.readline().decode('utf-8').strip()

            if linea:
                # Extrae los datos
                try:
                    (pitch, roll, esc1, esc2, esc3, esc4, yaw_angle_target, pitch_angle_target, 
                     roll_angle_target, throttle_target, pitch_angle_output, roll_angle_output) = parse_data(linea)
                     
                    # Almacena los últimos valores válidos
                    last_pitch, last_roll = pitch, roll
                    last_esc1, last_esc2, last_esc3, last_esc4 = esc1, esc2, esc3, esc4
                    last_yaw_angle_target, last_pitch_angle_target, last_roll_angle_target, last_throttle_target = yaw_angle_target, pitch_angle_target, roll_angle_target, throttle_target
                    last_pitch_angle_output, last_roll_angle_output = pitch_angle_output, roll_angle_output

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
            pitch_output_label.config(text=f"Pitch Output: {last_pitch_angle_output:.2f}")
            roll_output_label.config(text=f"Roll Output: {last_roll_angle_output:.2f}")

        except Exception as e:
            print(f"Error al leer datos del puerto serial: {e}")

    # Programa la próxima lectura
    root.after(50, read_serial_data)

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

# Crear un marco con borde azul y grueso
outer_frame = tk.Frame(root, bg="blue", bd=5)
outer_frame.grid(row=0, column=0, padx=10, pady=10)

frame = ttk.Frame(outer_frame, padding="10")
frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

# Fuente grande para las etiquetas
font_large = ("Helvetica", 16)

# Crear un marco para los datos PID de Pitch
pitch_frame = ttk.Frame(frame, padding="10")
pitch_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

# Etiquetas para valores de Pitch PID
pitch_label = ttk.Label(pitch_frame, text="Pitch: 0.00", font=font_large, foreground="blue")
pitch_label.grid(row=0, column=0, sticky=(tk.W))
pitch_output_label = ttk.Label(pitch_frame, text="Pitch Output: 0.00", font=font_large, foreground="blue")
pitch_output_label.grid(row=1, column=0, sticky=(tk.W))

# Etiquetas para valores de Pitch Target
pitch_target_label = ttk.Label(pitch_frame, text="Pitch Target: 0.00", font=font_large, foreground="blue")
pitch_target_label.grid(row=2, column=0, sticky=(tk.W))

# Crear un marco para los datos PID de Roll
roll_frame = ttk.Frame(frame, padding="10")
roll_frame.grid(row=3, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

# Etiquetas para valores de Roll PID
roll_label = ttk.Label(roll_frame, text="Roll: 0.00", font=font_large, foreground="green")
roll_label.grid(row=0, column=0, sticky=(tk.W))
roll_output_label = ttk.Label(roll_frame, text="Roll Output: 0.00", font=font_large, foreground="green")
roll_output_label.grid(row=1, column=0, sticky=(tk.W))

# Etiquetas para valores de Roll Target
roll_target_label = ttk.Label(roll_frame, text="Roll Target: 0.00", font=font_large, foreground="green")
roll_target_label.grid(row=2, column=0, sticky=(tk.W))

# Crear un marco para los datos de Yaw y Throttle
yaw_throttle_frame = ttk.Frame(frame, padding="10")
yaw_throttle_frame.grid(row=5, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

yaw_target_label = ttk.Label(yaw_throttle_frame, text="Yaw Target: 0.00", font=font_large, foreground="purple")
yaw_target_label.grid(row=0, column=0, sticky=(tk.W))
throttle_target_label = ttk.Label(yaw_throttle_frame, text="Throttle Target: 0.00", font=font_large, foreground="purple")
throttle_target_label.grid(row=1, column=0, sticky=(tk.W))

# Crear un marco para los valores de ESC
esc_frame = ttk.Frame(frame, padding="10")
esc_frame.grid(row=7, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

esc1_label = ttk.Label(esc_frame, text="ESC1: 0.00", font=font_large, foreground="red")
esc1_label.grid(row=0, column=0, sticky=(tk.W))
esc2_label = ttk.Label(esc_frame, text="ESC2: 0.00", font=font_large, foreground="red")
esc2_label.grid(row=1, column=0, sticky=(tk.W))
esc3_label = ttk.Label(esc_frame, text="ESC3: 0.00", font=font_large, foreground="red")
esc3_label.grid(row=2, column=0, sticky=(tk.W))
esc4_label = ttk.Label(esc_frame, text="ESC4: 0.00", font=font_large, foreground="red")
esc4_label.grid(row=3, column=0, sticky=(tk.W))



# Inicia la lectura del puerto serial
start_serial()

# Maneja el evento de cierre de la ventana
root.protocol("WM_DELETE_WINDOW", on_closing)

# Ejecuta la interfaz
root.mainloop()
