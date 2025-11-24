#!/usr/bin/env python3
import time
import threading

from gpiozero import DistanceSensor, RGBLED, Buzzer, Button, Motor
import board
import adafruit_dht
import BlynkLib

# ============================================================
#                     BLYNK SETUP
# ============================================================

BLYNK_AUTH = "4sQ3SVQwSDYxdYsS4g3k-dtEg9jEv5bF"

blynk = BlynkLib.Blynk(
    BLYNK_AUTH,
    server="blynk.cloud",
    port=80
)

# ============================================================
#                     PIN DEFINITIONS
# ============================================================

# Ultrasonic HC-SR04
ULTRASONIC_TRIGGER = 21
ULTRASONIC_ECHO = 20

# RGB LED
RED_PIN = 17
GREEN_PIN = 27
BLUE_PIN = 22

# Buzzer
BUZZER_PIN = 18

# DHT11
DHT_PIN = board.D26

# Fan (L293D)
EN1_PIN = 12
IN1_PIN = 24
IN2_PIN = 23

# Photoresistor (digital-style)
LDR_PIN = 5

# ARM/DISARM physical button
ARM_BUTTON_PIN = 16

# ============================================================
#                     OBJECTS
# ============================================================

distance_sensor = DistanceSensor(
    echo=ULTRASONIC_ECHO,
    trigger=ULTRASONIC_TRIGGER,
    max_distance=4.0
)

rgb = RGBLED(RED_PIN, GREEN_PIN, BLUE_PIN)
buzzer = Buzzer(BUZZER_PIN)
fan = Motor(forward=IN1_PIN, backward=IN2_PIN, enable=EN1_PIN, pwm=True)
ldr = Button(LDR_PIN, pull_up=True)
arm_button = Button(ARM_BUTTON_PIN, pull_up=True)

dht = adafruit_dht.DHT11(DHT_PIN)

# ============================================================
#                     CONSTANTS
# ============================================================

DISTANCE_CHANGE_THRESHOLD_CM = 10.0
TEMP_THRESHOLD_C = 28.0
FAN_ON_TIME_SEC = 5.0
FLASH_COUNT = 5
FLASH_DELAY = 0.1
MAX_DISTANCE_CM = 400.0

# ============================================================
#                     STATE VARIABLES
# ============================================================

armed = False
fan_running = False
fan_off_time = 0

remote_fan_override = False
remote_led_override = False

last_distance = None
last_temp = None
last_light_state = None

# NEW: flag so we know when we're in an alert (flashing red)
in_alert = False

# ============================================================
#                     HELPER FUNCTIONS
# ============================================================

def get_distance_cm():
    dist_m = distance_sensor.distance * distance_sensor.max_distance
    dist_cm = dist_m * 100
    if dist_cm <= 0 or dist_cm > MAX_DISTANCE_CM:
        return None
    return dist_cm

def read_temperature_c():
    try:
        t = dht.temperature
        return float(t) if t is not None else None
    except RuntimeError:
        return None

def beep_and_flash_red():
    """Flash red + buzzer a few times (used only when ARMED)."""
    global in_alert
    in_alert = True
    for _ in range(FLASH_COUNT):
        rgb.color = (1, 0, 0)
        buzzer.on()
        time.sleep(FLASH_DELAY)
        rgb.off()
        buzzer.off()
        time.sleep(FLASH_DELAY)
    in_alert = False

def is_dark():
    # If this feels reversed, change to: return not ldr.is_pressed
    return ldr.is_pressed

def fan_on():
    fan.forward(1.0)

def fan_off():
    fan.stop()

def set_led_baseline():
    """
    Sets the 'normal' LED state when we're NOT in an alert.
    - If remote LED override is on → always white
    - If ARMED → LED normally off (only alerts light it)
    - If DISARMED → LED is light-sensitive white lamp via LDR
    """
    if remote_led_override:
        rgb.color = (1, 1, 1)
        return

    if armed:
        # ARMED: baseline LED off (only red flash during alert)
        rgb.off()
    else:
        # DISARMED: LED follows light level
        if is_dark():
            rgb.color = (1, 1, 1)  # white in dark
        else:
            rgb.off()

def toggle_armed():
    global armed
    armed = not armed
    blynk.virtual_write(0, 1 if armed else 0)
    print("ARMED" if armed else "DISARMED")

# ============================================================
#                   BLYNK CALLBACKS
# ============================================================

def on_connect():
    print("Connected to Blynk!")
    # Sync widget states
    blynk.virtual_write(0, 1 if armed else 0)
    blynk.virtual_write(4, 1 if remote_fan_override else 0)
    blynk.virtual_write(5, 1 if remote_led_override else 0)

blynk.on("connected", on_connect)


@blynk.VIRTUAL_WRITE(0)   # ARM/DISARM switch
def v0_handler(value):
    global armed
    v = value[0] if isinstance(value, (list, tuple)) else value
    armed = (str(v) == "1")
    print("ARMED" if armed else "DISARMED")


@blynk.VIRTUAL_WRITE(4)   # Fan override
def v4_handler(value):
    global remote_fan_override
    v = value[0] if isinstance(value, (list, tuple)) else value
    remote_fan_override = (str(v) == "1")
    print("Fan override:", remote_fan_override)


@blynk.VIRTUAL_WRITE(5)   # LED override
def v5_handler(value):
    global remote_led_override
    v = value[0] if isinstance(value, (list, tuple)) else value
    remote_led_override = (str(v) == "1")
    print("LED override:", remote_led_override)

# ============================================================
#             PHYSICAL ARM/DISARM BUTTON
# ============================================================

arm_button.when_pressed = toggle_armed

# ============================================================
#                     MAIN LOOP
# ============================================================

def main_loop():
    global last_distance, last_temp, fan_running, fan_off_time, last_light_state

    print("System running. Press Ctrl+C to exit.")

    try:
        while True:
            # Allow Blynk to process network events
            blynk.run()

            # --- Ultrasonic Motion (ARMED only) ---
            distance = get_distance_cm()
            if distance is not None:
                blynk.virtual_write(2, round(distance, 1))

                if armed:
                    if last_distance is not None:
                        delta = abs(distance - last_distance)
                        if delta >= DISTANCE_CHANGE_THRESHOLD_CM:
                            print("Motion detected!")
                            beep_and_flash_red()
                last_distance = distance

            # --- Temperature + Fan ---
            temp = read_temperature_c()
            if temp is not None:
                blynk.virtual_write(1, round(temp, 1))

                if (temp >= TEMP_THRESHOLD_C or remote_fan_override) and not fan_running:
                    fan_on()
                    fan_running = True
                    fan_off_time = time.time() + FAN_ON_TIME_SEC

            if fan_running and not remote_fan_override and time.time() >= fan_off_time:
                fan_off()
                fan_running = False

            # --- Light Sensor + LED baseline ---
            dark = is_dark()
            state = "Dark" if dark else "Bright"
            blynk.virtual_write(3, state)

            # Only apply baseline LED behavior when NOT flashing red
            if not in_alert:
                set_led_baseline()

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Shutting down...")

    finally:
        fan_off()
        buzzer.off()
        rgb.off()
        print("Done.")

if __name__ == "__main__":
    main_loop()
