# suri2020
University of California, Merced. Summer Undergraduate Research Institute 2020. 

## Instructions
1. Burn the [AnchorNode.ino](AnchorNode/AnchorNode.ino) file to the anchor node. This is the arduino board with the cylindrical device. 
	1. Download the Ardunio IDE.
	2. Open [AnchorNode.ino](AnchorNode/AnchorNode.ino).
	3. Verify you are using the correct port and that the anchor node is plugged into a USB on your computer.
	4. Click upload to burn the code onto the anchor node.
2. Plug the anchor node into the Raspberry Pi 3 via USB. 
3. Power the Raspberry Pi 3. 
4. Connect the Raspberry Pi 3 to a computer via ethernet.
5. `ssh` into the Raspberry Pi 3.
	1. Observe the number written on top of the raspberry pi. This will be the missing number in the static ip address "'192.168.3.2**x**". 
		- For instance, if there is a "2" label on the pi, then the ip address will be 192.168.3.22.
	2. Run `ssh pi@192.168.3.2*` to log into the pi. Use the password "*vibration*"

## Materials
* [SM-24 Geophone Element](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/SM-24%20Brochure.pdf)
	* UB 10 Hz 375 Ohm
	* See also:
		* [Example code](https://github.com/olewolf/geophone)
* [Raspberry Pi 3 Model B](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/) 
	* V1.2
* [Raspberry Pi Zero W](https://www.raspberrypi.org/blog/raspberry-pi-zero-w-joins-family/)