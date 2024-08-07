import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Configura el puerto COM y la velocidad de baudios
puerto_com = 'COM18'  # Reemplaza con el puerto correcto
velocidad_baudios = 9600  # Asegúrate de que coincida con la configuración del Arduino

# Configura las figuras de matplotlib
fig_pitch_roll, ax_pitch_roll = plt.subplots()
fig_esc, ax_esc = plt.subplots()
fig_targets, ax_targets = plt.subplots()
fig_pitch_roll.suptitle('Pitch y Roll en tiempo real')
fig_esc.suptitle('Valores ESC en tiempo real')
fig_targets.suptitle('Valores Target en tiempo real')

xs = []
pitch_data = []
roll_data = []
esc1_data = []
esc2_data = []
esc3_data = []
esc4_data = []
yaw_angle_target_data = []
pitch_angle_target_data = []
roll_angle_target_data = []
throttle_target_data = []

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

def animate(i, line_pitch, line_roll, lines_esc, lines_targets, xs, pitch_data, roll_data, esc1_data, esc2_data, esc3_data, esc4_data, yaw_angle_target_data, pitch_angle_target_data, roll_angle_target_data, throttle_target_data):
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

        # Si no hay nueva línea o hubo un error, usa los últimos valores válidos
        pitch, roll = last_pitch, last_roll
        esc1, esc2, esc3, esc4 = last_esc1, last_esc2, last_esc3, last_esc4
        yaw_angle_target, pitch_angle_target = last_yaw_angle_target, last_pitch_angle_target
        roll_angle_target, throttle_target = last_roll_angle_target, last_throttle_target

        # Añade los datos a las listas
        xs.append(time.time())
        pitch_data.append(pitch)
        roll_data.append(roll)
        esc1_data.append(esc1)
        esc2_data.append(esc2)
        esc3_data.append(esc3)
        esc4_data.append(esc4)
        yaw_angle_target_data.append(yaw_angle_target)
        pitch_angle_target_data.append(pitch_angle_target)
        roll_angle_target_data.append(roll_angle_target)
        throttle_target_data.append(throttle_target)

        # Limita las listas a 20 elementos
        xs = xs[-20:]
        pitch_data = pitch_data[-20:]
        roll_data = roll_data[-20:]
        esc1_data = esc1_data[-20:]
        esc2_data = esc2_data[-20:]
        esc3_data = esc3_data[-20:]
        esc4_data = esc4_data[-20:]
        yaw_angle_target_data = yaw_angle_target_data[-20:]
        pitch_angle_target_data = pitch_angle_target_data[-20:]
        roll_angle_target_data = roll_angle_target_data[-20:]
        throttle_target_data = throttle_target_data[-20:]

        # Actualiza los datos de las líneas
        line_pitch.set_data(xs, pitch_data)
        line_roll.set_data(xs, roll_data)

        lines_esc[0].set_data(xs, esc1_data)
        lines_esc[1].set_data(xs, esc2_data)
        lines_esc[2].set_data(xs, esc3_data)
        lines_esc[3].set_data(xs, esc4_data)

        lines_targets[0].set_data(xs, yaw_angle_target_data)
        lines_targets[1].set_data(xs, pitch_angle_target_data)
        lines_targets[2].set_data(xs, roll_angle_target_data)
        lines_targets[3].set_data(xs, throttle_target_data)

        # Actualiza los límites de los ejes
        ax_pitch_roll.relim()
        ax_pitch_roll.autoscale_view()

        ax_esc.relim()
        ax_esc.autoscale_view()

        ax_targets.relim()
        ax_targets.autoscale_view()
    else:
        pass

def read_serial_data(port=puerto_com, baudrate=velocidad_baudios, timeout=1):
    global ser  # Utiliza la variable global ser
    try:
        # Abre el puerto serial
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"Puerto {ser.portstr} abierto")

    except serial.SerialException as e:
        print(f"Error al abrir el puerto: {e}")
    except Exception as e:
        print(f"Ocurrió un error: {e}")

def close_event(event):
    global ser
    print("Cerrando conexión serial...")
    if ser:
        ser.close()
    plt.close('all')

# Lee datos del puerto serial
read_serial_data()

# Inicializa las líneas
line_pitch, = ax_pitch_roll.plot([], [], label='Pitch')
line_roll, = ax_pitch_roll.plot([], [], label='Roll')
lines_esc = [
    ax_esc.plot([], [], label='ESC1')[0],
    ax_esc.plot([], [], label='ESC2')[0],
    ax_esc.plot([], [], label='ESC3')[0],
    ax_esc.plot([], [], label='ESC4')[0]
]

lines_targets = [
    ax_targets.plot([], [], label='Yaw Target')[0],
    ax_targets.plot([], [], label='Pitch Target')[0],
    ax_targets.plot([], [], label='Roll Target')[0],
    ax_targets.plot([], [], label='Throttle Target')[0]
]

ax_pitch_roll.legend(loc='upper left')
ax_pitch_roll.set_ylabel('Ángulo')
ax_pitch_roll.set_xlabel('Tiempo')

ax_esc.legend(loc='upper left')
ax_esc.set_ylabel('Velocidad')
ax_esc.set_xlabel('Tiempo')

ax_targets.legend(loc='upper left')
ax_targets.set_ylabel('Valor')
ax_targets.set_xlabel('Tiempo')

# Configura la animación para todas las figuras
ani_pitch_roll = animation.FuncAnimation(fig_pitch_roll, animate, fargs=(line_pitch, line_roll, lines_esc, lines_targets, xs, pitch_data, roll_data, esc1_data, esc2_data, esc3_data, esc4_data, yaw_angle_target_data, pitch_angle_target_data, roll_angle_target_data, throttle_target_data), interval=100, cache_frame_data=False)
ani_esc = animation.FuncAnimation(fig_esc, animate, fargs=(line_pitch, line_roll, lines_esc, lines_targets, xs, pitch_data, roll_data, esc1_data, esc2_data, esc3_data, esc4_data, yaw_angle_target_data, pitch_angle_target_data, roll_angle_target_data, throttle_target_data), interval=100, cache_frame_data=False)
ani_targets = animation.FuncAnimation(fig_targets, animate, fargs=(line_pitch, line_roll, lines_esc, lines_targets, xs, pitch_data, roll_data, esc1_data, esc2_data, esc3_data, esc4_data, yaw_angle_target_data, pitch_angle_target_data, roll_angle_target_data, throttle_target_data), interval=100, cache_frame_data=False)

# Conecta el evento de cierre de la ventana
fig_pitch_roll.canvas.mpl_connect('close_event', close_event)
fig_esc.canvas.mpl_connect('close_event', close_event)
fig_targets.canvas.mpl_connect('close_event', close_event)

plt.show()
