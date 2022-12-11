from machine import Pin

PIN_ROJO = 21
PIN_AZUL = 19
PIN_VERDE = 18

# LED Digital

led_rojo = Pin(PIN_ROJO, Pin.OUT)
led_verde = Pin(PIN_VERDE,Pin.OUT)
led_azul = Pin(PIN_AZUL,Pin.OUT)

# LED PWM

from machine import PWM

rojo_pwm = PWM(led_rojo)
rojo_pwm.duty(0)
rojo_pwm.duty(10)
rojo_pwm.duty(0)

verde_pwm = PWM(led_verde)
verde_pwm.duty(10)
verde_pwm.duty(0)

azul_pwm = PWM(led_azul)
azul_pwm.duty(10)
azul_pwm.duty(0)

def  colorRGB(r,g,b):
    rojo_pwm.duty(r)
    verde_pwm.duty(g)
    azul_pwm.duty(b)
    