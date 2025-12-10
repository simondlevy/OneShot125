SKETCH = Test

PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

build:
	arduino-cli compile --libraries $(HOME)/Documents/Arduino/libraries \
		--build-path build --fqbn $(FQBN) $(SKETCH).ino

dfu:
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/$(SKETCH).ino.bin

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3

clean:
	rm -rf build/
