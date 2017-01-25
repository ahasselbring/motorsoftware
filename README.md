# Alternative Motor Experiment Software

This repository contains an extension of the software for the motor experiment of the motor
board by the Institute for Control Systems at Hamburg University of Technology. The main goal
was to make it work with other operating systems than Windows.

The original software as provided in StudIP used two C mex functions: The actual motor communication
module and the [RTsync blockset](https://de.mathworks.com/matlabcentral/fileexchange/25043-rtsync-blockset-64-bit)
that slows down the simulation to real time. The compiled mex files have been provided only for Windows.
The motor communication module has been developed at ICS and has been provided including source code
whereas there is no source code available for RTsync.

# Usage

It is assumed that you have MATLAB already installed. The mex files were compiled for Linux 64 Bit
on an Intel Core i7-6700HQ. If something does not work, you need to compile them by running:

```bash
mex -O Motor001.cpp
mex -O RTsync.c
```

The mex files have to be copied to the correct locations, i.e. Motor001.mex* to Motor/
and RTsync.mex* to Motor/RTsync/.

After doing this, all Simulink projects should work like they do under Windows. You probably have to enter 0 as COM port.

Do not hesitate to ask me (e.g. by creating an issue) if something does not work or you
have suggestions for improvements.
