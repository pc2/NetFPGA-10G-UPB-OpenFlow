The NetFPGA-10G UPB OpenFlow Switch
===================================


Welcome to the NetFPGA-10G-UPB-OpenFlow repository.

Please read the provided [documentation](docs/documentation.pdf) carefully before
using this code.

This repository contains the NetFPGA-10G UPB Framework along with two projects:
 * upb_l2switch: A simple layer 2 network switch based on the NetFPGA-10G card.
 * upb_openflowswitch: The hardware implementation which is part of an OpenFlow Switch.
A modified version of Open vSwitch is required (available [here](https://github.com/pc2/ovs)).

Please note that the NetFPGA-10G UPB Framework is not compatible with the
Stanford NetFPGA-10G Framework.

### Feature Overview

 * Jumbo Frame Support
 * Large output queues using the external QDR2-SRAM memory
 * Flow Control using ethernet pause frames
 * Driver ported to Linux New API
 * Support for multiple cards in one host PC
 * Support for Samtec high speed port to interconnect multiple cards

### Requirements & Prerequisites
To build the projects you will need the following software (different versions
might work):
 * Ubuntu 14.04
 * Linux Kernel 3.14
 * Synplify Premier DP 2012.09-SP1
 * Xilinx ISE 14.7
 * Xilinx XPS 14.7

Licenses for the following IP cores are needed:
 * Xilinx 10G MAC
 * Xilinx Aurora


 This project was created as part of the project group [On-the-fly Networking
for Big Data](http://www.cs.uni-paderborn.de/fachgebiete/fachgebiet-rechnernetze/lehre/lehreteaching-ss14/pg-on-the-fly-networking-for-big-data.html)
at University of Paderborn.
