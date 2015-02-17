#!/usr/bin/python2

"""
 * Copyright (c) 2014, 2015 Felix Wallaschek
 * felix@elektronenversand.de
 *
 * This file is part of the NetFPGA 10G UPB OpenFlow Switch project:
 *
 * Project Group "On-the-Fly Networking for Big Data"
 * SFB 901 "On-The-Fly Computing"
 *
 * University of Paderborn
 * Computer Engineering Group
 * Pohlweg 47 - 49
 * 33098 Paderborn
 * Germany
 *
 * 
 * This file is free code: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 2.1 as
 * published by the Free Software Foundation.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this project. If not, see <http://www.gnu.org/licenses/>.
 
 Parameter for this script is NetFPGA-10G interface.
"""

from tuple_calculator import tuple_funs
import subprocess
import sys
WRAXI = "../../../../../../contrib-projects/upb_openflowswitch/sw/wraxi"
RDAXI = "../../../../../../contrib-projects/upb_openflowswitch/sw/rdaxi"
dev = sys.argv[1]
num_entries = int(tuple_funs.td(subprocess.check_output("sudo "+RDAXI+" "+dev+" 0xB0000008",shell=True).strip().split("=")[1]))
print "Deleting CAM Contents. This might take a while..."
sys.stdout.write("   0/"+str(num_entries))
for i in range(0,num_entries):
    cmds = tuple_funs.gen_wr_commands(0,0,dev=dev,cmd = WRAXI, hashh=i)
    sys.stdout.write("\b\b\b\b\b"+("\b"*len(str(num_entries)))+str(i+1).rjust(4)+"/"+str(num_entries))
    sys.stdout.flush()
    for cmd in cmds:
        subprocess.check_output(cmd.split())
print ""
