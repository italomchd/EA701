# BitDogLab / Raspberry Pi Pico — MicroPython
# PWM na GPIO0 com controle por joystick (incremental/latch) e ajuste fino nos botões,
# exibindo Freq, Duty e Alvo no OLED SSD1306 (I2C0: SDA=GPIO2, SCL=GPIO3).

from machine import Pin, PWM, ADC, I2C
import time
from ssd1306 import SSD1306_I2C

# ---------- PINOS ----------
PWM_PIN = 0       # GPIO0 -> saída PWM
BTN_A   = 5       # Botão A (PULL_UP)
BTN_B   = 6       # Botão B (PULL_UP)
ADC_X   = 26      # Joystick X -> frequência
ADC_Y   = 27      # Joystick Y -> duty

# OLED (I2C0 em GPIO2/3)
I2C_SDA = 2       # SDA
I2C_SCL = 3       # SCL

# ---------- LIMITES / PASSOS ----------
FREQ_MIN = 10           # Hz
FREQ_MAX = 20000        # Hz
JOY_FREQ_STEP = 100     # Hz por “empurrão” no X
JOY_DUTY_STEP = 5       # % por “empurrão” no Y

FINE_STEP_FREQ = 10     # +/– 10 Hz nos botões
FINE_STEP_DUTY = 10     # +/– 10% nos botões

# “zona morta” do joystick e repetição ao segurar
DEADZONE = 8000                 # ± da região central (0..65535)
REPEAT_START_MS = 350           # demora p/ primeira repetição ao segurar
REPEAT_INTERVAL_MS = 120        # intervalo entre repetições segurando

# ---------- HELPERS ----------
def clamp(x, lo, hi):
    return hi if x > hi else lo if x < lo else x

def pct_to_u16(p):
    return int(clamp(p, 0, 100) * 65535 / 100)

# ---------- ESTADO ----------
freq = 1000     # Hz inicial
duty = 50       # % inicial
active_target = "FREQ"   # último parâmetro alterado: "FREQ" ou "DUTY"

# estado do joystick p/ repetição
x_dir = 0; y_dir = 0
x_hold_t0 = None; y_hold_t0 = None
x_last_step = 0; y_last_step = 0

# botões (com debounce simples)
btn_a = Pin(BTN_A, Pin.IN, Pin.PULL_UP)
btn_b = Pin(BTN_B, Pin.IN, Pin.PULL_UP)
last_a = btn_a.value()
last_b = btn_b.value()
DEBOUNCE_MS = 25
last_btn_ms = 0

# PWM
pwm = PWM(Pin(PWM_PIN))
pwm.freq(freq)
pwm.duty_u16(pct_to_u16(duty))

adc_x = ADC(ADC_X)
adc_y = ADC(ADC_Y)

# OLED
i2c = I2C(1, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)
last_oled_ms = 0
OLED_INTERVAL_MS = 100

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
        if x_dir == 0:  # borda: neutro -> ativo
            freq = clamp(freq + dir_x * JOY_FREQ_STEP, FREQ_MIN, FREQ_MAX)
            active_target = "FREQ"
            x_hold_t0 = now
            x_last_step = now
        else:
            # repetição se mantiver fora do centro
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
        if y_dir == 0:  # borda
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
        # bordas de descida
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

    # ---- OLED: mostra a cada ~100 ms ----
    if time.ticks_diff(now, last_oled_ms) >= OLED_INTERVAL_MS:
        oled.fill(0)
        oled.text("PWM GPIO0", 0, 0)
        oled.text("Freq: {:>5}Hz".format(int(freq)), 0, 16)
        oled.text("Duty: {:>3}%".format(int(duty)), 0, 28)
        oled.text("Alvo: {}".format(active_target), 0, 40)
        oled.text("A:+10  B:-10", 0, 52)
        oled.show()
        last_oled_ms = now

    time.sleep_ms(15)
