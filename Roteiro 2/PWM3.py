# BitDogLab / Raspberry Pi Pico — MicroPython
# PWM na GPIO0 com controle por joystick (incremental/latch) e ajuste fino nos botões.
# Probe digital na GPIO1 com IRQ (subida/descida) para medir Freq e Duty.
# OLED SSD1306 (I2C1: SDA=GPIO14, SCL=GPIO15) exibe valores programados e medidos.

from machine import Pin, PWM, ADC, I2C, disable_irq, enable_irq
import time
from ssd1306 import SSD1306_I2C

# ---------- PINOS ----------
PWM_PIN = 0        # GPIO0 -> saída PWM
PROBE_PIN = 1      # GPIO1 -> entrada do probe (jumper GPIO0 -> GPIO1)
BTN_A   = 5        # Botão A (PULL_UP)
BTN_B   = 6        # Botão B (PULL_UP)
ADC_X   = 26       # Joystick X -> frequência
ADC_Y   = 27       # Joystick Y -> duty

# OLED (I2C1 em GPIO2/3)
I2C_SDA = 2
I2C_SCL = 3

# ---------- LIMITES / PASSOS ----------
FREQ_MIN = 10            # Hz
FREQ_MAX = 20000         # Hz
JOY_FREQ_STEP = 100      # Hz por “empurrão” no X
JOY_DUTY_STEP = 5        # % por “empurrão” no Y

FINE_STEP_FREQ = 10      # +/– 10 Hz nos botões
FINE_STEP_DUTY = 10      # +/– 10% nos botões

# “zona morta” do joystick e repetição ao segurar
DEADZONE = 8000
REPEAT_START_MS = 350
REPEAT_INTERVAL_MS = 120

# ---------- HELPERS ----------
def clamp(x, lo, hi):
    return hi if x > hi else lo if x < lo else x

def pct_to_u16(p):
    return int(clamp(p, 0, 100) * 65535 / 100)

# ---------- ESTADO PWM/CONTROLES ----------
freq = 1000     # Hz inicial
duty = 50       # % inicial
active_target = "FREQ"

x_dir = 0; y_dir = 0
x_hold_t0 = None; y_hold_t0 = None
x_last_step = 0; y_last_step = 0

btn_a = Pin(BTN_A, Pin.IN, Pin.PULL_UP)
btn_b = Pin(BTN_B, Pin.IN, Pin.PULL_UP)
last_a = btn_a.value()
last_b = btn_b.value()
DEBOUNCE_MS = 25
last_btn_ms = 0

# ---------- PWM ----------
pwm = PWM(Pin(PWM_PIN))
pwm.freq(freq)
pwm.duty_u16(pct_to_u16(duty))

adc_x = ADC(ADC_X)
adc_y = ADC(ADC_Y)

# ---------- OLED ----------
i2c = I2C(1, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)
last_oled_ms = 0
OLED_INTERVAL_MS = 100

# ---------- PROBE DIGITAL (GPIO2) ----------
probe = Pin(PROBE_PIN, Pin.IN, Pin.PULL_DOWN)

# Variáveis compartilhadas com IRQ (evite alocações na ISR)
_last_edge_t = None       # timestamp da última borda (us)
_high_time_us = 0         # último tempo em nível alto (us)
_low_time_us  = 0         # último tempo em nível baixo (us)
_period_us    = 0         # período calculado (us)
_new_period   = False     # flag de nova medida (alto+baixo)
_edges_total  = 0         # total de bordas (subida+descida) para freq por contagem

def _edge_irq(pin):
    # ISR: captura tempos alto/baixo e conta bordas
    # NÃO criar strings/objetos aqui!
    global _last_edge_t, _high_time_us, _low_time_us, _period_us, _new_period, _edges_total
    now = time.ticks_us()
    level = pin.value()   # 1 = após borda de subida, 0 = após borda de descida

    _edges_total += 1

    if _last_edge_t is not None:
        dt = time.ticks_diff(now, _last_edge_t)
        if level == 1:
            # subiu agora -> acabou LOW
            _low_time_us = dt
        else:
            # desceu agora -> acabou HIGH
            _high_time_us = dt

        if _high_time_us > 0 and _low_time_us > 0:
            _period_us = _high_time_us + _low_time_us
            _new_period = True

    _last_edge_t = now

probe.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_edge_irq)

# Janela para frequência por contagem de bordas
WINDOW_MS = 250
win_t0 = time.ticks_ms()
edges_at_win_start = 0

# ---------- LOOP ----------
while True:
    now = time.ticks_ms()

    # ---- JOYSTICK X -> FREQUÊNCIA (incremental/latch) ----
    x_raw = adc_x.read_u16()
    if x_raw > 32768 + DEADZONE:
        dir_x = +1
    elif x_raw < 32768 - DEADZONE:
        dir_x = -1
    else:
        dir_x = 0

    if dir_x != 0:
        if x_dir == 0:
            freq = clamp(freq + dir_x * JOY_FREQ_STEP, FREQ_MIN, FREQ_MAX)
            active_target = "FREQ"
            x_hold_t0 = now
            x_last_step = now
        else:
            if time.ticks_diff(now, x_hold_t0) >= REPEAT_START_MS and \
               time.ticks_diff(now, x_last_step) >= REPEAT_INTERVAL_MS:
                freq = clamp(freq + dir_x * JOY_FREQ_STEP, FREQ_MIN, FREQ_MAX)
                active_target = "FREQ"
                x_last_step = now
    else:
        x_hold_t0 = None
    x_dir = dir_x

    # ---- JOYSTICK Y -> DUTY (incremental/latch) ----
    y_raw = adc_y.read_u16()
    if y_raw > 32768 + DEADZONE:
        dir_y = +1
    elif y_raw < 32768 - DEADZONE:
        dir_y = -1
    else:
        dir_y = 0

    if dir_y != 0:
        if y_dir == 0:
            duty = clamp(duty + dir_y * JOY_DUTY_STEP, 0, 100)
            active_target = "DUTY"
            y_hold_t0 = now
            y_last_step = now
        else:
            if time.ticks_diff(now, y_hold_t0) >= REPEAT_START_MS and \
               time.ticks_diff(now, y_last_step) >= REPEAT_INTERVAL_MS:
                duty = clamp(duty + dir_y * JOY_DUTY_STEP, 0, 100)
                active_target = "DUTY"
                y_last_step = now
    else:
        y_hold_t0 = None
    y_dir = dir_y

    # ---- BOTÕES A/B -> AJUSTE FINO (±10) no alvo ativo ----
    a = btn_a.value()
    b = btn_b.value()
    if time.ticks_diff(now, last_btn_ms) > DEBOUNCE_MS:
        if last_a == 1 and a == 0:
            if active_target == "FREQ":
                freq = clamp(freq + FINE_STEP_FREQ, FREQ_MIN, FREQ_MAX)
            else:
                duty = clamp(duty + FINE_STEP_DUTY, 0, 100)
            last_btn_ms = now
        if last_b == 1 and b == 0:
            if active_target == "FREQ":
                freq = clamp(freq - FINE_STEP_FREQ, FREQ_MIN, FREQ_MAX)
            else:
                duty = clamp(duty - FINE_STEP_DUTY, 0, 100)
            last_btn_ms = now
    last_a, last_b = a, b

    # ---- APLICA NO PWM ----
    pwm.freq(int(freq))
    pwm.duty_u16(pct_to_u16(duty))

    # ---- FREQUÊNCIA POR CONTAGEM DE BORDAS (média de janela) ----
    freq_count = None
    if time.ticks_diff(now, win_t0) >= WINDOW_MS:
        irq_state = disable_irq()
        edges_snapshot = _edges_total
        enable_irq(irq_state)

        edges_in_window = edges_snapshot - edges_at_win_start
        edges_at_win_start = edges_snapshot

        # Cada ciclo completo tem 2 bordas => ciclos = edges/2
        window_s = WINDOW_MS / 1000.0
        freq_count = (edges_in_window / 2.0) / window_s
        win_t0 = now

    # ---- DUTY e Freq por período (a partir de tempos alto/baixo) ----
    duty_meas = None
    freq_meas = None
    irq_state = disable_irq()
    if _new_period and _period_us > 0:
        high_us = _high_time_us
        low_us = _low_time_us
        period_us = _period_us
        _new_period = False   # consome a medida
        enable_irq(irq_state)

        duty_meas = int(100 * high_us / period_us)
        freq_meas = 1_000_000.0 / period_us
    else:
        enable_irq(irq_state)

    # Fallback: se ainda não há período, use frequência por contagem
    if freq_meas is None and freq_count is not None:
        freq_meas = freq_count

    # ---- OLED: mostra a cada ~100 ms ----
    if time.ticks_diff(now, last_oled_ms) >= OLED_INTERVAL_MS:
        oled.fill(0)
        # Programado
        oled.text("PWM GPIO0", 0, 0)
        oled.text("F_prog:{:>5}Hz".format(int(freq)), 0, 12)
        oled.text("D_prog:{:>3}%".format(int(duty)), 0, 22)
        # Medido
        if freq_meas is not None:
            oled.text("F_med: {:>5.0f}Hz".format(freq_meas), 0, 36)
        else:
            oled.text("F_med:   ---", 0, 36)
        if duty_meas is not None:
            oled.text("D_med: {:>3}%".format(int(duty_meas)), 0, 46)
        else:
            oled.text("D_med:  ---", 0, 46)
        oled.text("Opcao sel.: {}".format(active_target), 0, 56)
        oled.show()
        last_oled_ms = now

    time.sleep_ms(15)
