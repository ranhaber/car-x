#!/usr/bin/env python3
import os

from .basic import _Basic_class

_GPIO_BACKEND = os.environ.get(
    "ROBOT_HAT_GPIO_BACKEND", "gpiozero"
).strip().lower()

if _GPIO_BACKEND == "rock4d":
    import gpiod
elif _GPIO_BACKEND == "gpiozero":
    import gpiozero  # https://gpiozero.readthedocs.io/en/latest/installing.html
    from gpiozero import OutputDevice, InputDevice, Button
else:
    raise ValueError(
        "ROBOT_HAT_GPIO_BACKEND must be either 'gpiozero' or 'rock4d'"
    )


# Raspberry Pi BCM number -> ROCK 4D gpiochip and line offset. The mappings
# follow the common 40-pin header's physical pin positions.
_ROCK4D_GPIO_MAP = {
    4: (1, 19),   # physical pin 7
    5: (3, 2),    # physical pin 29, Robot HAT MCU reset
    6: (1, 17),   # physical pin 31
    8: (1, 15),   # physical pin 24
    12: (1, 29),  # physical pin 32
    13: (1, 18),  # physical pin 33
    16: (1, 28),  # physical pin 36
    17: (1, 20),  # physical pin 11
    19: (1, 26),  # physical pin 35
    20: (1, 27),  # physical pin 38
    21: (1, 24),  # physical pin 40
    22: (1, 21),  # physical pin 15, ultrasonic echo
    23: (2, 14),  # physical pin 16, left motor direction
    24: (2, 15),  # physical pin 18, right motor direction
    25: (2, 31),  # physical pin 22
    26: (3, 3),   # physical pin 37
    27: (2, 16),  # physical pin 13, ultrasonic trigger
}


class _Rock4dDevice:
    """Small gpiozero-compatible wrapper around libgpiod v1."""

    def __init__(self, pin, output, pull=None, active_state=None):
        if pin not in _ROCK4D_GPIO_MAP:
            raise ValueError(f"BCM GPIO {pin} has no ROCK 4D mapping")

        chip_num, offset = _ROCK4D_GPIO_MAP[pin]
        self.pin = pin
        self._active_high = active_state is not False
        self._chip = gpiod.Chip(f"gpiochip{chip_num}")
        self._line = self._chip.get_line(offset)

        if output:
            self._line.request(
                consumer="robot_hat",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0],
            )
        else:
            flags = 0
            if pull == Pin.PULL_UP:
                flags = getattr(gpiod, "LINE_REQ_FLAG_BIAS_PULL_UP", 0)
            elif pull == Pin.PULL_DOWN:
                flags = getattr(gpiod, "LINE_REQ_FLAG_BIAS_PULL_DOWN", 0)
            elif pull is None:
                flags = getattr(gpiod, "LINE_REQ_FLAG_BIAS_DISABLE", 0)
            self._line.request(
                consumer="robot_hat",
                type=gpiod.LINE_REQ_DIR_IN,
                flags=flags,
            )

    @property
    def value(self):
        raw = self._line.get_value()
        return raw if self._active_high else 1 - raw

    def on(self):
        self._line.set_value(1 if self._active_high else 0)

    def off(self):
        self._line.set_value(0 if self._active_high else 1)

    def close(self):
        if self._line is not None:
            self._line.release()
            self._line = None
        if self._chip is not None:
            self._chip.close()
            self._chip = None


class Pin(_Basic_class):
    """Pin manipulation class"""

    OUT = 0x01
    """Pin mode output"""
    IN = 0x02
    """Pin mode input"""

    PULL_UP = 0x11
    """Pin internal pull up"""
    PULL_DOWN = 0x12
    """Pin internal pull down"""
    PULL_NONE = None
    """Pin internal pull none"""

    IRQ_FALLING = 0x21
    """Pin interrupt falling"""
    IRQ_RISING = 0x22
    """Pin interrupt falling"""
    IRQ_RISING_FALLING = 0x23
    """Pin interrupt both rising and falling"""

    _dict = {
        "D0": 17,
        "D1": 4,  # Changed
        "D2": 27,
        "D3": 22,
        "D4": 23,
        "D5": 24,
        "D6": 25,  # Removed
        "D7": 4,  # Removed
        "D8": 5,  # Removed
        "D9": 6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21,
        "SW": 25,  # Changed
        "USER": 25,
        "LED": 26,
        "BOARD_TYPE": 12,
        "RST": 16,
        "BLEINT": 13,
        "BLERST": 20,
        "MCURST": 5,  # Changed
        "CE": 8,
    }

    def __init__(self, pin, mode=None, pull=None, active_state:bool=None, *args, **kwargs):
        """
        Initialize a pin

        :param pin: pin number of Raspberry Pi
        :type pin: int/str
        :param mode: pin mode(IN/OUT)
        :type mode: int
        :param pull: pin pull up/down(PUD_UP/PUD_DOWN/PUD_NONE)
        :type pull: int
        :param active_state: active state of pin,  
                            If True, when the hardware pin state is HIGH, the software pin is HIGH. 
                            If False, the input polarity is reversed
        :type active_state: bool or None
        """
        super().__init__(*args, **kwargs)

        # parse pin
        if isinstance(pin, str):
            if pin not in self.dict().keys():
                raise ValueError(
                    f'Pin should be in {self._dict.keys()}, not "{pin}"')
            self._board_name = pin
            self._pin_num = self.dict()[pin]
        elif isinstance(pin, int):
            if pin not in self.dict().values():
                raise ValueError(
                    f'Pin should be in {self._dict.values()}, not "{pin}"')
            self._board_name = {i for i in self._dict if self._dict[i] == pin}
            self._pin_num = pin
        else:
            raise ValueError(
                f'Pin should be in {self._dict.keys()}, not "{pin}"')
        

        # setup
        self._value = 0
        self.gpio = None
        self.setup(mode, pull, active_state)
        self._info("Pin init finished.")

    def close(self):
        self.gpio.close()

    def deinit(self):
        self.gpio.close()
        if _GPIO_BACKEND == "gpiozero":
            self.gpio.pin_factory.close()

    def setup(self, mode, pull=None, active_state=None):
        """
        Setup the pin

        :param mode: pin mode(IN/OUT)
        :type mode: int
        :param pull: pin pull up/down(PUD_UP/PUD_DOWN/PUD_NONE)
        :type pull: int
        """
        # check mode
        if mode in [None, self.OUT, self.IN]:
            self._mode = mode
        else:
            raise ValueError(
                f'mode param error, should be None, Pin.OUT, Pin.IN')
        # check pull
        if pull in [self.PULL_NONE, self.PULL_DOWN, self.PULL_UP]:
            self._pull = pull
        else:
            raise ValueError(
                f'pull param error, should be None, Pin.PULL_NONE, Pin.PULL_DOWN, Pin.PULL_UP'
            )
        #
        if self.gpio != None:
            if self.gpio.pin != None:
                self.gpio.close()
        #
        if _GPIO_BACKEND == "rock4d":
            self.gpio = _Rock4dDevice(
                self._pin_num,
                output=mode in [None, self.OUT],
                pull=pull,
                active_state=active_state,
            )
        elif mode in [None, self.OUT]:
            self.gpio = OutputDevice(self._pin_num)
        else:
            if pull == self.PULL_UP:
                self.gpio = InputDevice(self._pin_num, pull_up=True, active_state=None)
            elif pull == self.PULL_DOWN:
                self.gpio = InputDevice(self._pin_num, pull_up=False, active_state=None)
            else:
                self.gpio = InputDevice(self._pin_num, pull_up=None, active_state=active_state)

    def dict(self, _dict=None):
        """
        Set/get the pin dictionary

        :param _dict: pin dictionary, leave it empty to get the dictionary
        :type _dict: dict
        :return: pin dictionary
        :rtype: dict
        """
        if _dict == None:
            return self._dict
        else:
            if not isinstance(_dict, dict):
                raise ValueError(
                    f'Argument should be a pin dictionary like {{"my pin": ezblock.Pin.cpu.GPIO17}}, not {_dict}'
                )
            self._dict = _dict

    def __call__(self, value):
        """
        Set/get the pin value

        :param value: pin value, leave it empty to get the value(0/1)
        :type value: int
        :return: pin value(0/1)
        :rtype: int
        """
        return self.value(value)

    def value(self, value: bool = None):
        """
        Set/get the pin value

        :param value: pin value, leave it empty to get the value(0/1)
        :type value: int
        :return: pin value(0/1)
        :rtype: int
        """
        if value == None:
            if self._mode in [None, self.OUT]:
                self.setup(self.IN)
            result = self.gpio.value
            self._debug(f"read pin {self.gpio.pin}: {result}")
            return result
        else:
            if self._mode in [self.IN]:
                self.setup(self.OUT)
            if bool(value):
                value = 1
                self.gpio.on()
            else:
                value = 0
                self.gpio.off()
            return value

    def on(self):
        """
        Set pin on(high)

        :return: pin value(1)
        :rtype: int
        """
        return self.value(1)

    def off(self):
        """
        Set pin off(low)

        :return: pin value(0)
        :rtype: int
        """
        return self.value(0)

    def high(self):
        """
        Set pin high(1)

        :return: pin value(1)
        :rtype: int
        """
        return self.on()

    def low(self):
        """
        Set pin low(0)

        :return: pin value(0)
        :rtype: int
        """
        return self.off()

    def irq(self, handler, trigger, bouncetime=200, pull=None):
        """
        Set the pin interrupt

        :param handler: interrupt handler callback function
        :type handler: function
        :param trigger: interrupt trigger(RISING, FALLING, RISING_FALLING)
        :type trigger: int
        :param bouncetime: interrupt bouncetime in miliseconds
        :type bouncetime: int
        """
        if _GPIO_BACKEND == "rock4d":
            raise NotImplementedError(
                "GPIO edge callbacks are not implemented for the ROCK 4D backend"
            )

        # check trigger
        if trigger not in [
                self.IRQ_FALLING, self.IRQ_RISING, self.IRQ_RISING_FALLING
        ]:
            raise ValueError(
                f'trigger param error, should be None, Pin.IRQ_FALLING, Pin.IRQ_RISING, Pin.IRQ_RISING_FALLING'
            )

        # check pull
        if pull in [self.PULL_NONE, self.PULL_DOWN, self.PULL_UP]:
            self._pull = pull
            if pull == self.PULL_UP:
                _pull_up = True
            else:
                _pull_up = False
        else:
            raise ValueError(
                f'pull param error, should be None, Pin.PULL_NONE, Pin.PULL_DOWN, Pin.PULL_UP'
            )
        #
        pressed_handler = None
        released_handler = None
        #
        if not isinstance(self.gpio, Button):
            if self.gpio != None:
                self.gpio.close()
            self.gpio = Button(pin=self._pin_num,
                               pull_up=_pull_up,
                               bounce_time=float(bouncetime / 1000))
            self._bouncetime = bouncetime
        else:
            if bouncetime != self._bouncetime:
                pressed_handler = self.gpio.when_pressed
                released_handler = self.gpio.when_released
                self.gpio.close()
                self.gpio = Button(pin=self._pin_num,
                                   pull_up=_pull_up,
                                   bounce_time=float(bouncetime / 1000))
                self._bouncetime = bouncetime
        #
        if trigger in [None, self.IRQ_FALLING]:
            pressed_handler = handler
        elif trigger in [self.IRQ_RISING]:
            released_handler = handler
        elif trigger in [self.IRQ_RISING_FALLING]:
            pressed_handler = handler
            released_handler = handler
        #
        if pressed_handler is not None:
            self.gpio.when_pressed = pressed_handler
        if released_handler is not None:
            self.gpio.when_released = released_handler

    def name(self):
        """
        Get the pin name

        :return: pin name
        :rtype: str
        """
        return f"GPIO{self._pin_num}"
