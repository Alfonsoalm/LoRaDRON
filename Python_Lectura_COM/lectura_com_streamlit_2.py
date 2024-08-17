import serial
import queue
import threading
import time
import streamlit as st
import plotly.graph_objects as go
from collections import deque

# Configuración del puerto COM y velocidad de baudios
puerto_com = 'COM22'
velocidad_baudios = 9600

ser = None  # Inicializa ser a None globalmente
data_queue = queue.Queue()

# Datos iniciales vacíos
pitch_data = deque(maxlen=100)  # Limitar a los últimos 100 puntos
pitch_target_data = deque(maxlen=100)
pitch_output_data = deque(maxlen=100)

roll_data = deque(maxlen=100)
roll_target_data = deque(maxlen=100)
roll_output_data = deque(maxlen=100)

esc1_data = deque(maxlen=100)
esc2_data = deque(maxlen=100)
esc3_data = deque(maxlen=100)
esc4_data = deque(maxlen=100)

time_data = deque(maxlen=100)

# Tiempo máximo de visualización en segundos
window_size = 10  # Ventana de tiempo para mostrar en la gráfica
sampling_interval = 0.1  # Intervalo en segundos para los datos

def parse_data(line):
    data_mapping = {
        'PITCH': 0.0,
        'ROLL': 0.0,
        'PITCH_target': 0.0,
        'ROLL_target': 0.0,
        'ESC1': 0.0,
        'ESC2': 0.0,
        'ESC3': 0.0,
        'ESC4': 0.0,
        'PITCH_output': 0.0,
        'ROLL_output': 0.0,
    }

    pairs = line.split(',')
    
    for pair in pairs:
        if ':' in pair:
            key, value_str = pair.split(':', 1)
            key = key.strip()
            value_str = value_str.strip()
            
            if key in data_mapping:
                try:
                    data_mapping[key] = float(value_str)
                except ValueError:
                    print(f"Error al convertir el valor para {key} en la línea: {line}")

    return (data_mapping['PITCH'],
            data_mapping['ROLL'],
            data_mapping['PITCH_target'],
            data_mapping['ROLL_target'],
            data_mapping['ESC1'],
            data_mapping['ESC2'],
            data_mapping['ESC3'],
            data_mapping['ESC4'],
            data_mapping['PITCH_output'],
            data_mapping['ROLL_output'])

def read_serial_data():
    global ser, data_queue

    while True:
        try:
            if ser and ser.is_open:
                linea = ser.readline().decode('utf-8').strip()
                if linea:
                    print(f"Datos leídos del puerto: {linea}")  # Imprimir datos en crudo
                    data = parse_data(linea)
                    if data:
                        print(f"Datos procesados: {data}")  # Imprimir datos procesados
                        data_queue.put(data)
            time.sleep(sampling_interval)  # Intervalo de lectura ajustado
        except Exception as e:
            print(f"Error en el hilo de lectura del puerto serial: {e}")
            # Intenta reiniciar la conexión si hay un error
            ser.close()
            time.sleep(2)  # Espera un poco antes de intentar reiniciar
            start_serial()

def start_serial():
    global ser
    try:
        ser = serial.Serial(puerto_com, velocidad_baudios, timeout=1)
        print(f"Puerto {ser.portstr} abierto")
    except serial.SerialException as e:
        print(f"Error al abrir el puerto: {e}")
    except PermissionError as e:
        print(f"Permiso denegado al abrir el puerto: {e}")
    except OSError as e:
        print(f"Error del sistema operativo al abrir el puerto: {e}")
    except Exception as e:
        print(f"Ocurrió un error inesperado: {e}")

def update_graph_data():
    global pitch_data, pitch_target_data, roll_data, roll_target_data
    global esc1_data, esc2_data, esc3_data, esc4_data, time_data

    # Procesar los datos
    if not data_queue.empty():
        data = data_queue.get()
        if data:
            pitch, roll, pitch_target, roll_target, esc1, esc2, esc3, esc4, pitch_output, roll_output = data

            # Añadir nuevos datos
            current_time = time.time()
            time_data.append(current_time)
            pitch_data.append(pitch)
            pitch_target_data.append(pitch_target)
            pitch_output_data.append(pitch_output)
            roll_data.append(roll)
            roll_target_data.append(roll_target)
            roll_output_data.append(roll_output)
            esc1_data.append(esc1)
            esc2_data.append(esc2)
            esc3_data.append(esc3)
            esc4_data.append(esc4)

            # Limitar el número de datos en la gráfica para mostrar una ventana de tiempo fija
            while time_data and (current_time - time_data[0]) > window_size:
                time_data.popleft()
                pitch_data.popleft()
                pitch_target_data.popleft()
                pitch_output_data.popleft()
                roll_data.popleft()
                roll_target_data.popleft()
                roll_output_data.popleft()
                esc1_data.popleft()
                esc2_data.popleft()
                esc3_data.popleft()
                esc4_data.popleft()

def plot_pitch_graph():
    global time_data

    # Crear la figura para Pitch
    fig = go.Figure()

    fig.add_trace(go.Scatter(x=list(time_data), y=list(pitch_data), mode='lines', name='Pitch', line=dict(color='blue')))
    fig.add_trace(go.Scatter(x=list(time_data), y=list(pitch_target_data), mode='lines', name='Pitch Target', line=dict(color='orange')))
    fig.add_trace(go.Scatter(x=list(time_data), y=list(pitch_output_data), mode='lines', name='Pitch Output', line=dict(color='cyan')))

    fig.update_yaxes(range=[-80, 80], title_text="Ángulo (°)")
    fig.update_layout(title_text="Pitch vs Pitch Target", xaxis_title="Tiempo (s)")

    return fig

def plot_roll_graph():
    global time_data

    # Crear la figura para Roll
    fig = go.Figure()

    fig.add_trace(go.Scatter(x=list(time_data), y=list(roll_data), mode='lines', name='Roll', line=dict(color='green')))
    fig.add_trace(go.Scatter(x=list(time_data), y=list(roll_target_data), mode='lines', name='Roll Target', line=dict(color='red')))
    fig.add_trace(go.Scatter(x=list(time_data), y=list(roll_output_data), mode='lines', name='Roll Output', line=dict(color='purple')))

    fig.update_yaxes(range=[-80, 80], title_text="Ángulo (°)")
    fig.update_layout(title_text="Roll vs Roll Target", xaxis_title="Tiempo (s)")

    return fig

def plot_esc_graph():
    global time_data

    # Crear la figura para ESCs
    fig = go.Figure()

    fig.add_trace(go.Scatter(x=list(time_data), y=list(esc1_data), mode='lines', name='ESC1', line=dict(color='cyan')))
    fig.add_trace(go.Scatter(x=list(time_data), y=list(esc2_data), mode='lines', name='ESC2', line=dict(color='magenta')))
    fig.add_trace(go.Scatter(x=list(time_data), y=list(esc3_data), mode='lines', name='ESC3', line=dict(color='brown')))
    fig.add_trace(go.Scatter(x=list(time_data), y=list(esc4_data), mode='lines', name='ESC4', line=dict(color='black')))

    fig.update_yaxes(title_text="Valor (%)")
    fig.update_layout(title_text="ESCs", xaxis_title="Tiempo (s)")

    return fig

def main():
    # Configuración de Streamlit
    st.title("Monitor de Datos en Tiempo Real")

    # Iniciar el hilo de lectura serial
    serial_thread = threading.Thread(target=read_serial_data, daemon=True)
    serial_thread.start()

    # Configurar las gráficas
    while True:
        update_graph_data()

        st.plotly_chart(plot_pitch_graph(), use_container_width=True)
        st.plotly_chart(plot_roll_graph(), use_container_width=True)
        st.plotly_chart(plot_esc_graph(), use_container_width=True)

        # Pausa para evitar sobrecargar el CPU
        time.sleep(sampling_interval)

if __name__ == "__main__":
    start_serial()
    main()
