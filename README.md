HomeControl
===========

This is the sourcecode and plans for my home automation system.

Content
=======

- `homecontrol`: Sources and schematics for a simple AtMega based home automation controller with 8 24VDC digital inputs, 8 24VDC digital outputs (open collector), RS232 and 2 230VAC sense inputs.
- `pixtend`: Sources for the main controller, based on an PiXtend v2-S with Raspberry 3B, written in Python. Connects via RS232 and MQTT to the network.

Installation Pixtend OS
=======================

- Install Raspbian OS (64bit)
   - `sudo rpi-imager`: Enable SSH, set hostname, username, password
- Setup Rasbian, install software
  ```
  # Enable SPI, UART, console bootup, set locale
  sudo raspi-config
  sudo apt install vim screen git gpiod libgpiod-dev python3-dev python3-pip python3-paho-mqtt
  ```
- Install Pixtend Python library
  ```
  git clone https://git.kontron-electronics.de/sw/ked/raspberry-pi/pixtend-v2/pplv2.git
  cd pplv2
  git checkout 0.1.6-dev
  python setup.py bdist_wheel
  # Install into venv
  python -m venv ~/.venv
  source ~/.venv/bin/activate
  pip install dist/pixtendlibv2-0.1.6-py3-none-any.whl
  ```
- Install pixcontrol script
  ```
  # Install prerequisites
  . ~/.venv/bin/activate
  pip install pyserial paho-mqtt
  cd ~/homecontrol/pixtend
  sudo cp pixcontrol.service /etc/systemd/system/
  sudo systemctl enable pixcontrol
  ```


License
=======

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
