# FPGA RSCPT UP5K

Demo / Sample Verilog code for AtomIDE + APIO for IceStorm/IceTools for Lattice Semi up5k
device on the HDL-0108-RSCPT from OshaBlue.

Experimental code to demo the HDL-0108-RSCPT (OshaBlue) 8-Channel rapid scan
ultrasound for NDT hardware.

It demonstrates on the Lattice Semi up5k sg48 device:

FIFO, PLL, and some basic Verilog IO and pin handling.

It's adaptable to various up5k demo hardware platforms also.  With some modifications.



## Quick Start

Look at:
- Making sure apio is completely installed (install_apio.sh/bat)
- <file>.pcf to see pin allocations for the HDL-0108-RSCPT pin usage
- top.v for the entire demo implementation

Programming:
- Upload the compiled image to your HDL-0108-RSCPT by simply programming your
FTDI USB-to-Serial(SPI) (C232HM EDHSL-0) cable to identify itself as the device
identified in apio.ini








## References

This demo code works pretty well for our initial needs; something good to build
from.

It owes its functional existence to many references and developers.  
Please see, references, components, and tools at, for example:

- The amazing: http://www.clifford.at/icestorm/
- The excellent: https://atom.io/packages/apio-ide and https://github.com/FPGAwars/apio-ide/wiki
- The awesome: https://tinyfpga.com/b-series-guide.html (etc)
- Full Suite of Solutions at: Lattice Semi: http://www.latticesemi.com/
- Great & Fundamental hardware solution building block at Upduino (search) is a great reference: https://github.com/gtjennings1/UPDuino_v2_0
- Plenty more ... to be added ... also please see comments in the code




## Introduction

There is a lot of flexibility here.  Many options for improvements and timing
adjustments.  This demo happens to work well for initial needs at this time.



## Code Status

It's fair to call this experimental, pre-pre-alpha, if anything.  Plenty of room
for clean up, plenty of room to clean up comments and MACRO/Conditional as well.

Comments in their excruciating conditional detail retained, at least in early
commits, for reference.  Maybe you can suggest a better to do that.




## Collected Examples

Several collected examples exist in the third party folders in this project,
typically module examples.  They are copyright their authors and collected
as examples to support project choices.  Let me know if any issues with having
them included here.  It facilitates learning and development here.



## License

Whatever is custom written here is under MIT License, copyrighted as noted, as
included or listed here in these files or license.

Most of the key modules and components are licensed by/to and copyrighted to
other authors.  Please see individual files.

Everything else is copyright or licensed as listed, is copyrighted by its
original authors or contributors, and is noted as such.

Thanks to all developers who make these projects possible.
