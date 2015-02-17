#!/usr/bin/python

"""
 * Copyright (c) 2014, 2015 Michael Lass
 * bevan@bi-co.net
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
 * 
"""

from cmd import Cmd

class CLI(Cmd):
    active = 0L
    offset = 0xa0000000
    prompt = "TCAM> "
    
    def __init__(self):
        Cmd.__init__(self)
        while True:
            try:
                self.cmdloop()
                break
            except KeyboardInterrupt:
                print "\nInterrupt"

    def print_instruction(self,value,addr):
        print "sudo ./wraxi /dev/nf10a 0x%0.8x 0x%0.8x" % (self.offset + (addr << 2), value)

    def do_activate (self,entry):
        entry = eval(entry)
        self.active |= 1 << entry
        addr = 0x03000 + int(entry / 32)
        value = (self.active >> (int(entry / 32) * 32)) & 0xffffffff
        self.print_instruction(value, addr)

    def do_deactivate (self,entry):
        entry = eval(entry)
        self.active &= ~(1 << entry)
        addr = 0x03000 + int(entry / 32)
        value = (self.active >> (int(entry / 32) * 32)) & 0xffffffff
        self.print_instruction(value, addr)

    def do_insert_into_tcam (self, s):
        pos, bitstring, bitmask, action = map(lambda x: eval(x),s.split())
        
        srls = dict()
        for i in range(49):
            bitstring_part = (bitstring >> (i*5)) & 0x1f
            bitmask_part = (bitmask >> (i*5)) & 0x1f
            srls[i] = self.calc_srl(bitstring_part, bitmask_part)
        for i in range(2):
            for j in range(32)[::-1]:
                value = 0
                for k in range(32):
                    if (i*32+k in srls.keys()):
                        value |= srls[i*32+k][j] << k
                    else:
                        value |= 0 << k
                addr = 0x01000 + pos*4 + i
                self.print_instruction(value, addr)
        self.print_instruction(action, 0x02000 + pos)

    def calc_srl(self,bitstring, bitmask):
        value = dict()
        for i in range(32):
            value[i] = (i & bitmask) == (bitstring & bitmask)
        return value

    def do_exit(self,s):
        return True

    do_quit = do_exit

    def do_EOF(self,s):
        print
        return True

    def emptyline(self):
        pass

if (__name__=="__main__"):
    CLI()
