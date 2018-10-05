#!/usr/bin/env python

# Import Pixtend V2 class
from pixtendv2s import PiXtendV2S
import time
import datetime
import sys
import serial
import math

class Button(object):
    OFF        = 0
    SHORT      = 1
    LONG       = 2

    def __init__(self, initstate):
        self._state = initstate
        self._ignore_release = False

    @property
    def state(self):
        return self._state

    def ignore_release(self, longpress = True):
        self._ignore_release = self.LONG if longpress else self.SHORT

    @property
    def released(self):
        r = self._get_released()
        if self._ignore_release == self.LONG:
            r = self.OFF
            self._ignore_release = self.OFF
        if self._ignore_release == self.SHORT:
            r = self.LONG if r == self.LONG else self.OFF
            self._ignore_release = self.OFF
        return r

    def toggle(self):
        self.update( not self.state )

    def toggle_by(self, button):
        if self.state is False and button.pressed:
            self.update( True )
            button.ignore_release()
        elif self.state is True and button.released != self.OFF:
            self.update( False )

class ToggleButton(Button):
    def __init__(self, initstate = False):
        Button.__init__(self, initstate)
        self._time_pressed = None
        self._pressed = False
        self._released = self.OFF
        self._is_released = False

    def update(self, pressed):
        if self._state is False and pressed is True:
            self._pressed = True
            self._state = True
            self._time_pressed = time.time()
        if self._state is True  and time.time() - self._time_pressed >= 2.5 and self._is_released is False:
            self._released = self.LONG
            self._is_released = True
        if self._state is True  and pressed is False:
            if self._is_released:
                self._is_released = False
            else:
                self._released = self.SHORT
            self._state = False

    @property
    def output(self):
        return self._state

    @property
    def pressed(self):
        val = self._pressed
        self._pressed = False
        return val

    def _get_released(self):
        val = self._released
        self._pressed = False
        self._released = self.OFF
        return val

class PulseButton(Button):
    """ Creates a pulse on every state change. """

    def __init__(self, initstate = False):
        Button.__init__(self, initstate)
        self._pulse = False
        self._pulse_start = None
        self._released = self.OFF
    
    def update(self, pressed):
        if self._state != pressed and self._pulse == False:
            self._pulse = True
            self._pulse_start = time.time()
            self._state = pressed

    @property
    def pressed(self):
        return self._pulse

    def _get_released(self):
        val = self._released
        self._released = self.OFF
        return val
    
    @property
    def output(self):
        if self._pulse and time.time() - self._pulse_start > 0.5:
            self._pulse = False
            self._released = self.SHORT
        return self._pulse

class SwitchButton(Button):
    def __init__(self, initstate = None):
        Button.__init__(self, initstate)
        self._changed = False

    def update(self, pressed):
        if self._state is None:
            self._state = pressed
        else:
            if self._state != pressed:
                self._changed = True
            self._state = pressed

    @property
    def pressed(self):
        val = self._changed
        return val

    def _get_released(self):
        val = self.SHORT if self._changed else self.OFF
        self._changed = False
        return val

    @property
    def output(self):
        return self._state

class Lamp(object):
    OFF   = 0
    ON    = 1
    BLINK = 2

    def __init__(self, top = 1000):
        self._state = self.OFF
        self._counter = 0.0
        self._top = top

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        if value == self.BLINK and self._state != value:
            # Reset counter when we start blinking
            self._counter = 0.0
        self._state = value

    @property
    def output(self):
        if self._state == self.ON:
            return self._top
        if self._state == self.BLINK:
            fact = 0.5 - math.cos( self._counter ) / 2.0
            # Will be called ~ every 50ms. Let's blink once every 3s
            self._counter += math.pi / 30
            return int(fact * self._top)
        return 0

class HomeControl(object):

    PUMP_OFF   = 0
    PUMP_ON    = 1
    PUMP_TIMER = 2

    def __init__(self):
        self.p = PiXtendV2S()
        
        if self.p is None:
            raise Exception("Could not initialize PiXtend SPI control interface.")

        self.uart = serial.Serial('/dev/ttyS0', 9600, timeout=0)

        # Controls
        self.auto_open_door = False
        
        # Inputs
        self.switch_pump = self.PUMP_OFF
        self.switch_cellar = ToggleButton()
        self.switch_garden = ToggleButton()
        self.switch_shop_small = SwitchButton()
        self.switch_shop_large = SwitchButton()
        self.button_bell_upper  = ToggleButton()
        self.button_bell_ground = ToggleButton()

        # Outputs
        self.garden_pump = SwitchButton(False)
        self.garden_light = PulseButton(False)
        self.cellar_light = SwitchButton(False)
        self.shop_small_light = SwitchButton(False)
        self.shop_large_light = SwitchButton(False)
        self.door = SwitchButton(False)

        self.lamp_cellar = Lamp()
        self.lamp_garden = Lamp()

    def reset(self):
        """ Reset the outputs. """
        self.p.digital_out0 = self.p.OFF
        self.p.digital_out1 = self.p.OFF
        self.p.digital_out2 = self.p.OFF
        self.p.digital_out3 = self.p.OFF
        self.p.relay0 = self.p.OFF
        self.p.relay1 = self.p.OFF
        self.p.relay2 = self.p.OFF
        self.p.relay3 = self.p.OFF

    def shutdown(self):
        self.reset()
        
        self.p.watchdog = self.p.WDT_OFF

        time.sleep(0.5)
        
        self.p.close()
        self.p = None

    def setup(self):
        """ Initialize PiXtend. """

        # Setup GPIOs (0: Input, 1: Output, 2: DHT11, 3: DHT22)
        self.p.gpio0_ctrl = 0
        self.p.gpio1_ctrl = 0
        self.p.gpio2_ctrl = 0
        self.p.gpio3_ctrl = 0
        self.p.gpio_pullups_enable = True

        # Setup PWM outputs
        # PWM mode, enable A+B, 64x prescaler
        self.p.pwm0_ctrl0 = 121
        # TOP is 1000 (120Hz)
        self.p.pwm0_ctrl1 = 1000
        # Turn off LEDs
        self.p.pwm0a = 0
        self.p.pwm0b = 0

        #self.p.watchdog = 4000
        #self.p.state_let_off = True

        self.reset()

    def crc_ok(self):
        return self.p.crc_header_in_error is False and self.p.crc_data_in_error is False

    @property
    def any_cellar_light(self):
        return self.cellar_light.state or self.shop_small_light.state or self.shop_large_light.state

    def read_inputs(self):
        # Pool pump switch
        if self.p.analog_in0 < 1.0:
            self.switch_pump = self.PUMP_TIMER
        elif self.p.analog_in0 > 3.0:
            self.switch_pump = self.PUMP_ON
        else:
            self.switch_pump = self.PUMP_OFF

        self.switch_cellar.update( self.p.digital_in3 )
        self.switch_garden.update( self.p.digital_in2 )
        self.switch_shop_small.update( self.p.digital_in1 )
        self.switch_shop_large.update( self.p.digital_in0 )
        self.button_bell_upper.update( self.p.digital_in5 )
        self.button_bell_ground.update( self.p.digital_in4 )

    def control(self):
        # Pump control
        if self.switch_pump == self.PUMP_OFF:
            self.garden_pump.update( False )
        if self.switch_pump == self.PUMP_ON:
            self.garden_pump.update( True )
        if self.switch_pump == self.PUMP_TIMER:
            dt = datetime.datetime.now()
            if (dt.month >= 5 and dt.month < 10) and
               ((dt.hour >= 11 and dt.hour < 12) or (dt.hour >= 17 and dt.hour < 20)):
                self.garden_pump.update( True )
            else:
                self.garden_pump.update( False )

        # Garden light
        self.garden_light.toggle_by( self.switch_garden )

        # Cellar lights
        self.shop_large_light.toggle_by( self.switch_shop_large )
        self.shop_small_light.toggle_by( self.switch_shop_small )

        # Cellar central light
        cellar_released = self.switch_cellar.released
        if cellar_released == Button.SHORT:
            self.cellar_light.toggle()
        if cellar_released == Button.LONG:
            self.cellar_light.update( False )
            self.shop_large_light.update( False )
            self.shop_small_light.update( False )


    def update_outputs(self):
        # Relays
        self.p.relay0 = self.door.output
        self.p.relay1 = self.shop_large_light.output
        self.p.relay2 = self.shop_small_light.output
        self.p.relay3 = self.cellar_light.output

        self.p.digital_out0 = self.garden_pump.output
        self.p.digital_out1 = self.garden_light.output

        # Signal lamps
        if self.cellar_light.state is True:
            self.lamp_cellar.state = Lamp.ON
        elif self.any_cellar_light:
            self.lamp_cellar.state = Lamp.BLINK
        else:
            self.lamp_cellar.state = Lamp.OFF

        if self.garden_light.state is True:
            self.lamp_garden.state = Lamp.ON
        elif self.garden_pump.state is True:
            self.lamp_garden.state = Lamp.BLINK
        else:
            self.lamp_garden.state = Lamp.OFF

        self.p.pwm0a = self.lamp_cellar.output
        self.p.pwm0b = self.lamp_garden.output

# -----------------------------------------------------
# Main Program
# -----------------------------------------------------
c = HomeControl()

c.setup()

time.sleep(0.5)

while True:
    try:
        # Check if SPI communication is running and the received data is correct
        if c.crc_ok():
            
            c.read_inputs()
            
            c.control()

            c.update_outputs()

        else:
            print("")
            print("Communication error, the data from the microcontroller is not correct!")
            print("Leaving the application. Please check that the Raspberry Pi can communicate")
            print("with the microcontroller on the PiXtend V2 -S- board.")
            print("")
            
            c.shutdown()

            break
            
        # Wait some time, SPI communication will continue in the background
        time.sleep(0.05)

    except KeyboardInterrupt:
        # Keyboard interrupt caught, Ctrl + C, now clean up and leave program
        
        c.shutdown()
        
        break
