#!/usr/bin/env python
#
# Project:   HomeControl
# File:      pixcontrol.py
# Author:    Stefan Hepp <stefan@stefant.org>
#
# Copyright 2018 Stefan Hepp
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# Control loop implementation for my home controller.
#
# This software is intended to be run on a PiXtend v2-S board.
#

# Import Pixtend V2 class
from pixtendv2s import PiXtendV2S
import time
import datetime
import sys
import serial
import math
import paho.mqtt.client as mqtt

class Button(object):
    OFF        = 0
    SHORT      = 1
    LONG       = 2

    def __init__(self, initstate = False):
        self._state = initstate
        self._status = self.OFF

    @property
    def state(self):
        return self._state

    @property
    def pressed(self):
        return self._status

    def clear(self):
        self._status = self.OFF

class PushButton(Button):
    def __init__(self):
        Button.__init__(self)
        self._time_pressed = None
        self._is_released = False

    def update(self, pressed):
        if self._state is False and pressed is True:
            # Start timer
            self._time_pressed = time.time()
        if self._state is True  and time.time() - self._time_pressed >= 2.5 and self._is_released is False:
            self._status = self.LONG
            self._is_released = True
        if self._state is True  and pressed is False:
            if not self._is_released:
                self._status = self.SHORT
            self._is_released = False
        self._state = pressed

class SwitchButton(Button):
    def __init__(self):
        Button.__init__(self, None)

    def update(self, pressed):
        if self._state is not None and self._state != pressed:
            # Every state change is a press event
            self._status = self.SHORT
        self._state = pressed


class MQTTSerializer(object):
    def serialize(self, value):
        return value

    def deserialize(self, payload):
        return payload

class TableSerializer(MQTTSerializer):
    """ Translate values via a lookup table. """

    def __init__(self, table):
        self._table = table
        self._reverse_table = dict(zip(table.values(), table.keys()))

    def serialize(self, value):
        return self._table.get(value)

    def deserialize(self, payload):
        return self._reverse_table.get(payload)

class TimeSerializer(MQTTSerializer):
    def serialize(self, value):
        return value.strftime("%H:%M")

    def deserialize(self, payload):
        """ Split a H:M string and return a time object, or None if the format does not match """
        t = payload.split(":")
        if len(t) != 2:
            return None
        return datetime.time( int(t[0]), int(t[1]) )


class Signal(object):
    def __init__(self, initstate = False):
        self._input = initstate
        self._output = initstate
        self._publish = False
        self._changed = False
        self._translate = TableSerializer( { True: "ON", False: "OFF" } )

    @property
    def translate(self):
        return self._translate

    @translate.setter
    def translate(self, value):
        self._translate = TableSerializer( value )

    def on_message(self, client, userdata, msg):
        data = self._translate.deserialize( str( msg.payload ) )
        if data is not None:
            self.update(data, publish=False)

    def publish(self, client, topic, retain = True, subscribe = True, publish = True):
        self._mqtt = client
        self._topic = topic
        self._retain = retain
        self._publish = publish
        if subscribe:
            self._mqtt.subscribe(topic)
            self._mqtt.message_callback_add(self._topic, lambda c, u, m: self.on_message(c, u, m))
        if publish:
            data = self._translate.serialize(self._input)
            if data is not None:
                self._mqtt.publish(self._topic, data, retain = self._retain)

    def update(self, value, publish=True):
        if self._input is not None and self._input != value:
            self._changed = True
            # Publish MQTT data
            if publish and self._publish:
                data = self._translate.serialize(value)
                if data is not None:
                    self._mqtt.publish(self._topic, data, retain = self._retain)
        self._update(value)
        self._input = value

    def toggle(self):
        self.update(not self._input)

    @property
    def state(self):
        return self._input

    @property
    def changed(self):
        return self._changed

    def clear(self):
        self._changed = False

    @property
    def output(self):
        self._update_output()
        return self._output

    def _update(self, value):
        pass

    def _update_output(self):
        pass

class StateSignal(Signal):
    def __init__(self, initstate = False):
        Signal.__init__(self, initstate)

    def _update(self, value):
        self._output = value

class PulseSignal(Signal):
    """ Creates a pulse on every state change. """

    def __init__(self, duration = 0.5):
        Signal.__init__(self)
        self._pulse_start = None
        self.duration = duration
    
    def _update(self, value):
        if self._input != value:
            self._pulse_start = time.time()
            self._output = True

    def _update_output(self):
        if self._output and time.time() - self._pulse_start > self.duration:
            self._output = False

class ExtendSignal(Signal):
    """ Extends the length of an input pulse by some duration. """

    def __init__(self, duration = 1.0):
        Signal.__init__(self)
        self._pulse_start = None
        self.duration = duration
    
    def _update(self, value):
        if value:
            self._output = True
        elif self._input:
            # Start timeout for pulse extenstion
            self._pulse_start = time.time()

    def _update_output(self):
        if self._output and not self._input and time.time() - self._pulse_start > self.duration:
            self._output = False

class TimeProvider:
    def __init__(self):
        self.dtime = datetime.datetime.now()

    def update(self):
        self.dtime = datetime.datetime.now()

class TimeSignal(Signal):
    """ Trigger an output based on day time. """

    EARLIER_THAN = 0
    LATER_THAN   = 1

    def __init__(self, timeprovider, hour = 0, minute = 0, op = EARLIER_THAN):
        Signal.__init__(self, datetime.time(hour, minute))
        self._time = timeprovider
        self._op = op
        self._translate = TimeSerializer()

    def _update_output(self):
        if self._op == self.EARLIER_THAN:
            self._output = self._time.dtime.time() < self._input
        else:
            self._output = self._time.dtime.time() > self._input


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

    STAIR_OFF   = 0
    STAIR_ON    = 1
    STAIR_SENSE = 2

    ALL_OFF_NONE   = 0
    ALL_OFF_CELLAR = 1
    ALL_OFF_ALL    = 2

    # Command header
    UART_CMD_HEADER        = 0x70

    # Command Opcodes:
    # - Living room light. Value = 0: Off, 1: Light on, 2: Long press
    UART_CMD_LIVINGROOM    = 0x01
    # - Stair case light. Value = 0: Off, 1: Light on, 2: Stair long press
    UART_CMD_STAIRCASE     = 0x02
    # - Stair light. Value = 000000 | Stair Sense | Stair Light
    UART_CMD_STAIR         = 0x03
    # - All off. Value = 1: All off
    UART_CMD_ALL_OFF       = 0x04
    # - Status. Value = 000 | Sense | Stair Sense | Stair Light | Staircase | Livingroom
    UART_CMD_STATUS        = 0x05
    # - Sense notification. Value = 1: Motion detected
    UART_CMD_SENSE         = 0x06


    def __init__(self):
        self.p = PiXtendV2S()
        
        if self.p is None:
            raise Exception("Could not initialize PiXtend SPI control interface.")

        # Setup UART
        self.uart = serial.Serial('/dev/ttyS0', 9600, timeout=0)
        self.uart_buf = []

        self.now = TimeProvider()

        # Configuration
        self.auto_open_door = StateSignal()
        self.all_off_time = TimeSignal(self.now, 6, 30, TimeSignal.LATER_THAN)
        self.all_off_trigger = StateSignal()
        self.pump_interval_1_start = TimeSignal(self.now, 11, 0, TimeSignal.LATER_THAN)
        self.pump_interval_1_end   = TimeSignal(self.now, 12, 0, TimeSignal.EARLIER_THAN)
        self.pump_interval_2_start = TimeSignal(self.now, 17, 0, TimeSignal.LATER_THAN)
        self.pump_interval_2_end   = TimeSignal(self.now, 19, 0, TimeSignal.EARLIER_THAN)
        self.pump_interval_3_start = TimeSignal(self.now,  0, 0, TimeSignal.LATER_THAN)
        self.pump_interval_3_end   = TimeSignal(self.now,  0, 0, TimeSignal.EARLIER_THAN)

        # Inputs
        self.switch_pump = StateSignal(self.PUMP_OFF)
        self.switch_cellar = PushButton()
        self.switch_garden = PushButton()
        self.switch_shop_small = SwitchButton()
        self.switch_shop_large = SwitchButton()
        self.button_garden_bell  = StateSignal()
        self.button_garden_door = StateSignal()

        # HomeControl relay signals
        self.livingroom_light = StateSignal()
        self.staircase_light = StateSignal()
        self.stair_light = StateSignal(self.STAIR_OFF)
        self.stair_sensor = StateSignal()

        # MQTT IO signals
        self.mqtt_all_off = StateSignal(self.ALL_OFF_NONE)

        # Control signals
        self.pump_control = StateSignal(self.PUMP_TIMER)
        self.pump_override = StateSignal()
        self.pump_timer = StateSignal()

        # Outputs
        self.garden_pump = StateSignal()
        self.garden_light = PulseSignal()
        self.cellar_light = StateSignal()
        self.shop_small_light = StateSignal()
        self.shop_large_light = StateSignal()
        self.door = ExtendSignal(duration=3.0)

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

    def all_off(self, staircase=False):
        self.cellar_light.update( False )
        self.shop_large_light.update( False )
        self.shop_small_light.update( False )
        self.garden_light.update( False )
        if staircase:
            self.send_uart(self.UART_CMD_ALL_OFF, 1)

    def mqtt_setup(self):
        self.mqtt = mqtt.Client("Pixtend")
        self.mqtt.on_connect = lambda c, d, f, r: self.mqtt_connect(c, d, f, r)
        self.mqtt.on_message = lambda c, d, m: self.mqtt_message(c, d, m)
        self.mqtt.connect_async("data.home")
        self.mqtt.loop_start()

    def mqtt_stop(self):
        self.mqtt.loop_stop()

    def mqtt_connect(self, client, userdata, flags, rc):
        self.livingroom_light.publish('home/light/livingroom')
        self.staircase_light.publish('home/light/staircase')
        self.stair_light.translate = {self.STAIR_OFF: 'OFF', self.STAIR_ON: 'ON', self.STAIR_SENSE: 'SENSE'}
        self.stair_light.publish('home/light/stair')
        self.stair_sensor.translate = {True: 'TRIGGERED'}
        self.stair_sensor.publish('home/control/stair_sensor', subscribe=False, retain=False)

        self.garden_light.publish('home/light/garden')
        self.cellar_light.publish('home/light/cellar')
        self.shop_small_light.publish('home/light/shop_small')
        self.shop_large_light.publish('home/light/shop_large')

        self.button_garden_bell.publish('home/control/doorbell')
        self.door.publish('home/control/door')
        self.auto_open_door.publish('home/settings/door_auto_open')

        self.pump_control.translate = {self.PUMP_OFF: 'OFF', self.PUMP_ON: 'ON', self.PUMP_TIMER: 'TIMER'}
        self.pump_control.publish('home/pump/control')
        self.pump_override.publish('home/pump/override')
        self.pump_timer.publish('home/pump/timer', subscribe=False)
        self.garden_pump.publish('home/pump/status', subscribe=False)

        self.mqtt_all_off.translate = {self.ALL_OFF_CELLAR: 'CELLAR', self.ALL_OFF_ALL: 'ALL'}
        self.mqtt_all_off.publish('home/light/all_off', publish=False, retain=False)

        self.all_off_time.publish('home/control/all_off_time')
        self.pump_interval_1_start.publish('home/pump/interval_1_start')
        self.pump_interval_1_end.publish(  'home/pump/interval_1_end'  )
        self.pump_interval_2_start.publish('home/pump/interval_2_start')
        self.pump_interval_2_end.publish(  'home/pump/interval_2_end'  )
        self.pump_interval_3_start.publish('home/pump/interval_3_start')
        self.pump_interval_3_end.publish(  'home/pump/interval_3_end'  )

    def mqtt_message(self, client, userdata, msg):
        pass

    def read_inputs(self):
        # Pool pump switch
        if self.p.analog_in0 < 1.0:
            self.switch_pump.update( self.PUMP_TIMER )
        elif self.p.analog_in0 > 3.0:
            self.switch_pump.update( self.PUMP_ON )
        else:
            self.switch_pump.update( self.PUMP_OFF )

        self.switch_cellar.update( self.p.digital_in3 )
        self.switch_garden.update( self.p.digital_in2 )
        self.switch_shop_small.update( self.p.digital_in1 )
        self.switch_shop_large.update( self.p.digital_in0 )
        self.button_garden_bell.update( self.p.digital_in4 )
        self.button_garden_door.update( self.p.digital_in5 )

    def read_uart(self):
        # Read UART buffer into buffer
        self.uart_buf += list( self.uart.read(100) )
        # Scan UART buffer for messages
        while len( self.uart_buf ) > 1:
            cmd = ord(self.uart_buf[0])
            val = ord(self.uart_buf[1])
            if cmd & 0xF0 == self.UART_CMD_HEADER:
                cmd = cmd & 0x0F
                if cmd == self.UART_CMD_LIVINGROOM:
                    # Long press is ignored (not a push button)
                    self.livingroom_light.update( bool(val) )
                    # No need to send change event back
                    self.livingroom_light.clear()
                if cmd == self.UART_CMD_STAIRCASE:
                    if val < 2:
                        self.staircase_light.update( bool(val) )
                        # No need to send change event back
                        self.staircase_light.clear()
                    else:
                        # TODO define action for long press on outside staircase button
                        pass
                if cmd == self.UART_CMD_STAIR:
                    self.stair_light.update( val )
                    # No need to send change event back
                    self.stair_light.clear()
                if cmd == self.UART_CMD_ALL_OFF and val == 1:
                    self.all_off()
                if cmd == self.UART_CMD_STATUS:
                    self.livingroom_light.update( bool(val & 1) )
                    self.staircase_light.update( bool((val >> 1) & 1) )
                    self.stair_light.update( (val >> 2) & 3 )
                    self.stair_sensor.update( bool((val >> 4) & 1) )
                    # No need to send change event back
                    self.livingroom_light.clear()
                    self.staircase_light.clear()
                    self.stair_light.clear()
                    self.stair_sensor.clear()
                if cmd == self.UART_CMD_SENSE:
                    self.stair_sensor.update( val )

                # Take message from buffer
                self.uart_buf = self.uart_buf[2:]
            else:
                # No valid command, skip this byte
                self.uart_buf = self.uart_buf[1:]

    def send_uart(self, cmd, value):
        self.uart.write( chr(self.UART_CMD_HEADER + cmd) + chr( int( value ) ) )

    def update_timer(self):
        """ Return True if the pump timer is active, else False. """
        self.now.update()
        enabled = (self.pump_interval_1_start.output and self.pump_interval_1_end.output) or \
                  (self.pump_interval_2_start.output and self.pump_interval_2_end.output) or \
                  (self.pump_interval_3_start.output and self.pump_interval_3_end.output)
        self.pump_timer.update( enabled )

        # Update a state signal with the all-off timer state to trigger on positive edge.
        self.all_off_trigger.update( self.all_off_time.output )

    def control_lights(self):
        # Garden light
        if self.switch_garden.pressed == Button.SHORT:
            self.garden_light.toggle()
        if self.switch_garden.pressed == Button.LONG:
            self.pump_override.toggle()

        # Cellar lights
        if self.switch_shop_large.pressed == Button.SHORT:
            self.shop_large_light.toggle()

        if self.switch_shop_small.pressed == Button.SHORT:
            self.shop_small_light.toggle()

        # Cellar central light
        if self.switch_cellar.pressed == Button.SHORT:
            self.cellar_light.toggle()
        if self.switch_cellar.pressed == Button.LONG:
            self.all_off()

        # Send out changes via MQTT to HomeControl
        if self.livingroom_light.changed:
            self.send_uart( self.UART_CMD_LIVINGROOM, self.livingroom_light.output )
        if self.staircase_light.changed:
            self.send_uart( self.UART_CMD_STAIRCASE, self.staircase_light.output )
        if self.stair_light.changed:
            self.send_uart( self.UART_CMD_STAIR, self.stair_light.output )
        if self.stair_sensor.changed:
            # No action for stair_sensor detection. Clear signal.
            self.stair_sensor.update( False )

        # MQTT inputs
        if self.mqtt_all_off.state == self.ALL_OFF_CELLAR:
            self.all_off(staircase = False)
        if self.mqtt_all_off.state == self.ALL_OFF_ALL:
            self.all_off(staircase = True)
        # Clear MQTT trigger input
        self.mqtt_all_off.update(self.ALL_OFF_NONE)

        # Turn off all lights when the timer triggers
        if self.all_off_trigger.changed and self.all_off_trigger.state:
            self.all_off(staircase = True)

        # Clear handled events
        self.switch_garden.clear()
        self.switch_shop_large.clear()
        self.switch_shop_small.clear()
        self.switch_cellar.clear()
        self.livingroom_light.clear()
        self.staircase_light.clear()
        self.stair_light.clear()
        self.stair_sensor.clear()
        self.mqtt_all_off.clear()
        self.all_off_trigger.clear()

    def control_pump(self):
        # Pump manual switch override
        if self.switch_pump.changed:
            self.pump_control.update( self.switch_pump.output )

        # If the timer triggered, disable the timer override
        if self.pump_timer.changed:
            self.pump_override.update( False )

        # Control pump state
        if self.pump_control.output == self.PUMP_OFF:
            self.garden_pump.update( False )
        if self.pump_control.output == self.PUMP_ON:
            self.garden_pump.update( True )
        if self.pump_control.output == self.PUMP_TIMER:
            if self.pump_override.state:
                self.garden_pump.update( not self.pump_timer.output )
            else:
                self.garden_pump.update( self.pump_timer.output )

        # Clear handled events
        self.switch_pump.clear()
        self.pump_control.clear()
        self.pump_override.clear()
        self.pump_timer.clear()

    def control_door(self):
        # Door control
        if self.auto_open_door.output or True:
            if self.button_garden_bell.changed and not self.button_garden_door.output:
                self.door.update( self.button_garden_bell.output )
        elif self.auto_open_door.changed:
            # When turning off the automatic door opener, make sure we disable the relay.
            self.door.update(False)
        if self.button_garden_door.changed:
            self.door.update( self.button_garden_door.output )

        # Clear handled events
        self.button_garden_bell.clear()
        self.button_garden_door.clear()
        self.door.clear()
        self.auto_open_door.clear()

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
        elif self.garden_pump.state is True and not self.pump_timer.output is True:
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

c.mqtt_setup()

time.sleep(0.5)

while True:
    try:
        # Check if SPI communication is running and the received data is correct
        if c.crc_ok():
            
            c.read_inputs()
            
            c.read_uart()

            c.update_timer()

            c.control_lights()

            c.control_pump()

            c.control_door()

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
