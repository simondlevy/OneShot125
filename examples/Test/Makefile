SKETCH = Test

PORT = /dev/ttyACM0

FQBN = teensy:avr:teensy40

build:
	arduino-cli compile  --libraries ../..,../../.. --fqbn $(FQBN) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

upload:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200
