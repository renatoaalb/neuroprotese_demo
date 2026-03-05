import utils.imu_yostlabs_lara as imu_yostlabs_lara
import utils.quaternion_operations as quaternion_operations
import time
import keyboard
from math import pi
from stim_node import StimNode


# ──────────────────────────────────────────
#  CONFIGURACOES
# ──────────────────────────────────────────

PORT       = "COM11"
FREQ       = 50
N_FACTOR   = 0
GROUP_TIME = 0
STIM_MODE  = 0
CHANNEL_LF = []

CHANNELS = [1]

PULSE_WIDTH_OFF   = [0]
PULSE_CURRENT_OFF = [0]

PULSE_WIDTH_ON    = [100]
PULSE_CURRENT_ON  = [11]


# ──────────────────────────────────────────
#  ESTADOS
# ──────────────────────────────────────────

STATE_WAITING  = "waiting"
STATE_STIM_ON  = "stim_on"
STATE_STIM_OFF = "stim_off"
STATE_EXIT     = "exit"

# ──────────────────────────────────────────
#  INICIALIZACAO IMU
# ──────────────────────────────────────────



# ──────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────

def main():
    stim = StimNode(
        port=PORT,
        freq=FREQ,
        n_factor=N_FACTOR,
        group_time=GROUP_TIME,
        stim_mode=STIM_MODE,
        channel_lf=CHANNEL_LF,
    )

    stim.initialize_ccl(CHANNELS)
    stim.update_ccl(PULSE_WIDTH_OFF, PULSE_CURRENT_OFF)
    print("[INFO] Sistema inicializado com valores zerados.")
    print("-" * 45)
    print("  ESPACO -> iniciar estimulacao")
    print("  P      -> pausar (valores zerados)")
    print("  L      -> retomar estimulacao")
    print("  ESC    -> encerrar programa")
    print("-" * 45)
    print("[INFO] Aguardando ESPACO para iniciar...")

    state = STATE_WAITING

    # configuration imu

    imus = [9]
    input("Press Enter to start configuration")
    serial_port = imu_yostlabs_lara.initialize_dongle(imus)
    imu_yostlabs_lara.configure_imu(serial_port, imus)

    current_quaternion1 = None
    x = float()
    y = float()

    # start streaming
    input("Press Enter to start streaming")
    imu_yostlabs_lara.start_streaming(serial_port, imu_ids = imus, frequency = 100, timestamp = True)

    startTime = time.time()
    serial_port.reset_input_buffer()
    while state != STATE_EXIT:
        data = imu_yostlabs_lara.read_data(serial_port)
        if data is not None:
            quaternion1 = imu_yostlabs_lara.extract_data(data, type_of_data = 0, imu_id = imus[0])
            if quaternion1 is not None:
                current_quaternion1 = quaternion1
                
        if current_quaternion1 is not None:            
            euler_angle = quaternion_operations.euler_from_quaternion(current_quaternion1)
            
            x = euler_angle[0]
            y = euler_angle[1]

            if y >= 0:
                if abs(x) > (pi*0.5):
                    y = 180-y
            else:
                if abs(x) > (pi*0.5):
                    y = 180-y
                else:
                    y = 360+y
            
            #print(euler_angle)

            current_quaternion1 = None  
        
        match state:

            case "waiting":     # estado de espera, aperta espaço que entra no programad da estimulação
                if y > 270 or y < 20:
                    state = STATE_STIM_ON
                    stim.update_ccl(PULSE_WIDTH_ON, PULSE_CURRENT_ON)
                    print("[STIM ON]  Estimulacao iniciada.")
                    time.sleep(0.3)

            case "stim_on":     # estado de estimulação, quando entra no programa de estimulação
                stim.update_ccl(PULSE_WIDTH_ON, PULSE_CURRENT_ON)

                if keyboard.is_pressed("p") or (y < 270 or y > 20):    #condição de "pausa"
                    state = STATE_STIM_OFF
                    stim.update_ccl(PULSE_WIDTH_OFF, PULSE_CURRENT_OFF)
                    print("[PAUSADO]  Valores zerados. Pressione L para retomar.")
                    time.sleep(0.3)

                elif keyboard.is_pressed("esc"):    # condição de saida e encerramento
                    state = STATE_EXIT

            case "stim_off":        #condição de pausa
                stim.update_ccl(PULSE_WIDTH_OFF, PULSE_CURRENT_OFF)

                if keyboard.is_pressed("l") or (y > 270 or y < 20):    # condição de estimulação
                    state = STATE_STIM_ON
                    stim.update_ccl(PULSE_WIDTH_ON, PULSE_CURRENT_ON)
                    print("[STIM ON]  Estimulacao retomada.")
                    time.sleep(0.3)

                elif keyboard.is_pressed("esc"):    # condição de saida e encerramento
                    state = STATE_EXIT

        time.sleep(0.05)

    stim.stop_ccl() # encerra o programa e finaliza
    print("[INFO] Estimulacao encerrada. Programa finalizado.")

    # stop streaming
    imu_yostlabs_lara.stop_streaming(serial_port, imus)


if __name__ == "__main__":
    main()