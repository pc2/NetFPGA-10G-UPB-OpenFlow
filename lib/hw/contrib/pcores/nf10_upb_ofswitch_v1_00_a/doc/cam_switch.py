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

CAM Switch

Use this simple script to install CAM entries directly from captured packets.
Make sure to add the MAC adresses and port numbers of your computers 
below as scapy is not able to determine the interface a packet came from.
Parameters for this script are virtual network interfaces of NetFPGA card
(nf10av0 etc.)

WARNING:
There is a possibility that an unfortunate timing when interrupting this
script leaves the CAM Module in an inconsistent state. Be sure to only 
interrupt this script when no network traffic is taking place and reset 
the NetFPGA afterwards.
"""


WRAXI = "../../../../../../contrib-projects/upb_openflowswitch/sw/wraxi"
learnedMACs = {"0x0000c9fb8282":1,"0x0000c9c790ec":0, "0x0000c9c790f0":0, "0x0000c9fb8286":1}


from scapy.all import *
import sys
import threading
import time
import os
from tuple_calculator import tuple_funs
from threading import Lock

lock = Lock()

def dottedQuadToNum(ip):
    "convert decimal dotted quad string to long integer"
    hexn = ''.join(["%02X" % long(i) for i in ip.split('.')])
    return str(long(hexn, 16))
    
def mon_pkt_callback(pkt):
    if not Ether in pkt:
        return
    tup = { "valid":1,
            "port":0,
            "vport":0,
            "dmac":"0x"+pkt[Ether].dst.replace(":",""),
            "smac":"0x"+pkt[Ether].src.replace(":",""),
            "typ":pkt[Ether].type,
            "vid":0,
            "pcp":0,
            "sip":0,
            "dip":0,
            "prot":0,
            "tos":0,
            "tsp":0,
            "tdp":0}
    if UDP in pkt:
        tup["tsp"]=pkt[UDP].sport
        tup["tdp"]=pkt[UDP].dport
    if TCP in pkt:
        tup["tsp"]=pkt[TCP].sport
        tup["tdp"]=pkt[TCP].dport
    if IP in pkt:
        tup["tos"]=pkt[IP].tos
        tup["sip"]=dottedQuadToNum(pkt[IP].src)
        tup["dip"]=dottedQuadToNum(pkt[IP].dst)
        tup["prot"]=pkt[IP].proto
    if ICMP in pkt:
        tup["tsp"]=pkt[ICMP].type
        tup["tdp"]=pkt[ICMP].code
        
    action = "0x00FF"
    if tup["smac"] in learnedMACs:
        tup["port"] = learnedMACs[tup["smac"]]
    if tup["dmac"] in learnedMACs:
        action = tuple_funs.th(1<<learnedMACs[tup["dmac"]])
        
    commands = tuple_funs.gen_wr_commands(action,tuple_funs.create_tuple(tup),cam_size=2048,cmd=WRAXI)
    lock.acquire()
    print "Installing Flow... Action: "+str(action) + "\t Tuple: " + str(tup)
    for line in commands:
        subprocess.check_output(line.split())
    lock.release()

threads=[]
try:
    for i in sys.argv[1:]:
        t = threading.Thread(target=sniff,kwargs={"store":0,"prn":mon_pkt_callback,"iface":i})
        t.daemon = True
        threads.append(t)
        t.start()
        time.sleep(1)
    
    
    while True:
        time.sleep(10)
except:
    print "Caught Exception. Shutting down..."
    lock.acquire()
