# Axpert King Inverter MQTT integration with Home Assistant

There are other integrations for Axpert inverters available. Some are quite old or are intended to be used in docker. None really fitted me so I ended up doing my own "simple" one.

Currently working with Axpert King Inverter (Rack), basically because it is what I have. It should be easy to adapt the "defs" file with the commands for other variants. Happy to support if interested.

**_This is provided with no warranty. Setting inverter wrong values can damage your inverter or be dangerous. Use at your own risk and only if you know what you are doing_**.

![image](https://github.com/user-attachments/assets/16d78560-a1d1-4f5b-b757-48ccf52b8e07)

## Features

- Automatic discovery in Home Assistant via the MQTT integration.
- Read only values: power, voltage, etc...
- Most configuration settings. Some settings, such as output type (i.e. three-phase output) are provided read-only in HASS for safety, as these should be really set at physical installation time. These can still be changed manually via command line.
- Command line access to the inverter commands, even if the service is running in the background.
- **Configuration mode**: By default, settings cannot be changed unless configuration PIN is provided in HASS. A random PIN is created on first run and stored in `inverter.conf` the first time the service is run. Leave it blank (not commented) to disable PIN. Please see [bugs](https://github.com/puzzle-star/axpert-inverter-hass/new/master?filename=README.md#bugs) below about PIN being recorded and readeble in Home Assistant.

## Installation

To install, just clone the repository and execute the installation helper.


### Requirements

 - A working `gcc` to compile the IO helper for `USBHID` connection (this will be done automatically by the install helper and with no external library dependecies, anyway)
 - The provided `systemd` service configuration files assue installation in `/opt/inverter`
 - The provided systemd service files assume the inverter is connected using the integrated USB HID port, not ther serial one.

### Install

```
git clone https://github.com/puzzle-star/axpert-inverter-hass /opt/inverter
/opt/bin/axpert-query --install
```

Edit the installed `/opt/inverter/etc/inverter.conf` file to provide your MQTT credentials. The service will fail to start untill you provide these, but once done, it should start automatically.

## Bugs

The configuration PIN is easily dicovered in Home Assistant by just looking into the history of the text field. It is intended to avoid acccidental activation of **configuration mode**, but if you want the PIN to be kept secret, please make sure co configure the Home Assistant recorder not to keep track of the `text.inverter_unlock_config_mode_pin` entity.
