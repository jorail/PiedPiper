# PiedPiper
Whistle speed control for model trains, using microcontroller, FFT tone analysis, outdoor compatible with on-board USB power supply 

The program purpose is to control a model train motor with independent power supply e.g. from USB power bank for outdoors. 
Following options exist:

a) Control by a whistle Morse code signals. Works with microphone input and FFT tone analysis in a Teensy 4.0 microcontroller

a) Switch Morse code input

a) SerialMonitor input commands by typing letters, requires connection via USB

The project is inspired by the project 'free your model train' (FYMT) proposed by Frei Softwarefreunde
at https://freie-software.org/free-your-model-train/

The code and layout are developed for Teensy 4.0 but can easily be modified for other micro controllers.
Sound evaluation and tone signal identification is performed by FFT analysis in the microprocessor.
Identified tone signals or Morse code commands from a switch are processed to commands for change of 
the speed level. New motor settings are transfered via pulse width modulation (PWM) output 
to a H-bridge motor control IC.

The idea for the FFT analysis for tone identification and part of the functions are based on 'Audio Tone Input' 
(toneinput.ino) by Tony DiCola, which is published with MIT License as part of the ardafruit 
learning guide and examples at http://learn.adafruit.com/fft-fun-with-fourier-transforms/. The corresponding 
function headings are marked by 'MIT License'.

Further explanations for the PiedPiper project including the electrical layout and images of the resulting 
circuit and wiring as well of its testing and application are provided in the [/docs](docs/)
folder.

An idea for tone identification by advanced sound signal processing with wavelet analysis 
(wavelib: https://github.com/rafat/wavelib) is not yet implemented in the PiedPiper code.

The continued development of the code for model train control by smartphone via WiFi and using an ESP32 
microcontroller is presented in this second repository: https://github.com/jorail/PiedPiperS

Please feel invited to pass your comments on github or via e-mail, if you have any suggestions for further 
improvement or new applications for PiedPiper.

Have fun with it and invite other to play with it together... Jo
