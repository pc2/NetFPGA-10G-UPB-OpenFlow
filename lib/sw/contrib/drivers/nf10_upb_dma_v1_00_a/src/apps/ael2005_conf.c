/*******************************************************************************
 *
 *  NetFPGA-10G http://www.netfpga.org
 *
 *  File:
 *        ael2005_conf.c
 *
 *  Project:
 *        nic
 *
 *  Author:
 *        M. Forconesi (Cambridge Team)
 *
 *        Modifications for the UPB OpenFlow Switch project:
 *         Jörg Niklas, osjsn@niklasfamily.de
 *         Project Group "On-the-Fly Networking for Big Data"
 *         Computer Engineering Group, University of Paderborn
 *
 *  Description:
 *        Configures the AEL2005 PHY chips on the NetFPGA-10G.
  *
 *  Copyright notice:
 *        Copyright (C) 2010, 2011 The Board of Trustees of The Leland Stanford
 *                                 Junior University
 *
 *        Modifications for the UPB OpenFlow Switch project:
 *          Copyright (c) 2014, 2015 Jörg Niklas, osjsn@niklasfamily.de
 *
 *  Licence:
 *        This file is part of the NetFPGA 10G development base package.
 *
 *        This file is free code: you can redistribute it and/or modify it under
 *        the terms of the GNU Lesser General Public License version 2.1 as
 *        published by the Free Software Foundation.
 *
 *        This package is distributed in the hope that it will be useful, but
 *        WITHOUT ANY WARRANTY; without even the implied warranty of
 *        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *        Lesser General Public License for more details.
 *
 *        You should have received a copy of the GNU Lesser General Public
 *        License along with the NetFPGA source package.  If not, see
 *        http://www.gnu.org/licenses/.
 *
 */

//#define DEBUG

#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "ael2005_conf.h"

#define MDIO_BASE_ADDR 0x7a000000   // Check the Xilinx XPS Address Space if the hw changed

int main(int argc, char* argv[]){
    int f;
    int port, dev;
    uint16_t value;
    char option;
    char port_mode;

    printf("==NetFPGA-10G==\nAEL2005 Initialization and Status Program\n");

	if (argc != 2) {
		printf("Usage: ael2005_conf netfpga_device\n");
		return 0;
	}

    //----------------------------------------------------
    //-- open nf10 file descriptor for all the fun stuff
    //----------------------------------------------------
    f = open(argv[1], O_RDWR);
    if(f < 0){
        perror("Error opening device");
        return 0;
    }
    system("clear");

    while(1) {
        printf("==Option Menu==\n\n");
        printf("[a] : Initialize all interfaces 10G mode\n");
        printf("[b] : Interface 0 (closer to PCIe pins) 10G mode\n");
        printf("[c] : Interface 1 10G mode\n");
        printf("[d] : Interface 2 10G mode\n");
        printf("[e] : Interface 3 10G mode\n");
        printf("[m] : Initialize all interfaces  1G mode\n");
        printf("[n] : Interface 0 (closer to PCIe pins)  1G mode\n");
        printf("[o] : Interface 1 1G mode\n");
        printf("[p] : Interface 2 1G mode\n");
        printf("[q] : Interface 3 1G mode\n");
        printf("[r] : Reset all PHY\n");
        printf("[s] : Dump current status\n");
        printf("[l] : PMA Loopback mode\n");
        printf("[x] : Exit this program\n");

        option = getchar();
        getchar();
        system("clear");
        if(option == 'a'){
            printf("**************\n10G Config Starts\n**************");fflush(stdout);
            for(port = 0; port < 4; port ++){
                if(port == 0) dev = 2;
                if(port == 1) dev = 1;
                if(port == 2) dev = 0;
                if(port == 3) dev = 3;
                value = 0;
                printf("\nPort number: %d\n", port);

                // Check if we have a 10GBASE-SR cable
                ael2005_i2c_read (f, dev, MODULE_DEV_ADDR, 0x3, &value);
                if((value >> 4) == 1) {port_mode = MODE_SR; printf("Detected MODE_SR\n");}
                else {port_mode = MODE_TWINAX; printf("Detected MODE_TWINAX\n");}

                fflush(stdout);
                ael2005_initialize(f, dev, port_mode);
                printf("Conf port: %d DONE\n", port);fflush(stdout);
            }
            printf("**************\nConfig Finished\n**************\n");fflush(stdout);
        }
        else if(option == 'b' || option == 'c' || option == 'd' || option == 'e'){
            printf("**************\n10G Config Starts\n**************");fflush(stdout);
            if(option == 'b') {port = 0; dev = 2;}
            if(option == 'c') {port = 1; dev = 1;}
            if(option == 'd') {port = 2; dev = 0;}
            if(option == 'e') {port = 3; dev = 3;}
            value = 0;
            printf("\nPort number: %d\n", port);

            // Check if we have a 10GBASE-SR cable
            ael2005_i2c_read (f, dev, MODULE_DEV_ADDR, 0x3, &value);
            if((value >> 4) == 1) {port_mode = MODE_SR; printf("Detected MODE_SR\n");}
            else {port_mode = MODE_TWINAX; printf("Detected MODE_TWINAX\n");}

            fflush(stdout);
            ael2005_initialize(f, dev, port_mode);
            printf("conf port: %d DONE\n", port);fflush(stdout);
            printf("**************\nConfig Finished\n**************\n");fflush(stdout);
        }
        else if(option == 'm'){
            printf("**************\n1G Config Starts\n**************");fflush(stdout);
            for(port = 0; port < 4; port ++){
                if(port == 0) dev = 2;
                if(port == 1) dev = 1;
                if(port == 2) dev = 0;
                if(port == 3) dev = 3;
                value = 0;

                printf("\nPort number: %d\n", port);
                ael2005_initialize_1g(f, dev, 0);
                printf("conf port: %d DONE\n", port);fflush(stdout);
            }
            printf("**************\nConfig Finished\n**************\n");fflush(stdout);
        }
        else if(option == 'n' || option == 'o' || option == 'p' || option == 'q'){
            printf("**************\n1G Config Starts\n**************");fflush(stdout);
            if(option == 'n') {port = 0; dev = 2;}
            if(option == 'o') {port = 1; dev = 1;}
            if(option == 'p') {port = 2; dev = 0;}
            if(option == 'q') {port = 3; dev = 3;}
            value = 0;

            printf("\nPort number: %d\n", port);
            ael2005_initialize_1g(f, dev, 0);
            printf("conf port: %d DONE\n", port);fflush(stdout);
            printf("**************\nConfig Finished\n**************\n");fflush(stdout);
        }
        else if (option == 'r'){
            printf("**************\nReset Starts\n**************");fflush(stdout);
            for(port = 0; port < 4; port ++){
                if(port == 0) dev = 2;
                if(port == 1) dev = 1;
                if(port == 2) dev = 0;
                if(port == 3) dev = 3;
                value = 0;

                printf("\nPort number: %d\n", port);fflush(stdout);
                ael2005_reset(f, dev);
                printf("Reset port: %d DONE\n", port);fflush(stdout);
            }
            printf("**************\nReset Finished\n**************\n");fflush(stdout);
        }
        else if (option == 's'){
            test_status(f);
        }
        else if (option == 'l'){
            printf("**************\nPMA Loopback\n**************");fflush(stdout);
            for(port = 0; port < 4; port ++){
                if(port == 0) dev = 2;
                if(port == 1) dev = 1;
                if(port == 2) dev = 0;
                if(port == 3) dev = 3;
                value = 0;

                printf("\nPort number: %d\n", port);fflush(stdout);
                ael2005_sys_loopback(f, dev);
                ael2005_sleep(2);
                printf("Port: %d in loopback mode DONE\n", port);fflush(stdout);
            }
            printf("**************\nPMA Loopback Finished\n**************");fflush(stdout);
        }
        else if (option == 'x'){
            break;
        }
    }

    printf("\n");
    close(f);
    return 0;
}

int ael2005_sys_loopback(int file_ptr, uint32_t dev){
    ael2005_write(file_ptr, dev, PMA_MDIO_DEVICE_ADDRESS, PMA_CTL_REGISTER_ADDRESS, PMA_SYS_LOOPBACK_REGISTER_VALUE);
    return 0;
}

int ael2005_reset(int file_ptr, uint32_t dev){
    ael2005_write(file_ptr, dev, PMA_MDIO_DEVICE_ADDRESS, PMA_CTL_REGISTER_ADDRESS, PMA_RESET_REGISTER_VALUE);
    return 0;
}

int ael2005_initialize(int file_ptr, uint32_t dev, int mode){
    int size, i;

    printf("AEL2005 Initialization Start..\n");fflush(stdout);
    // Step 1
    printf("Step 1..\n");fflush(stdout);
    size = sizeof(reset) / sizeof(uint16_t);
    for(i = 0; i < size; i+=2) ael2005_write(file_ptr, dev, PMA_MDIO_DEVICE_ADDRESS, reset[i], reset[i+1]);
    ael2005_sleep(50);

    // Step 2
    printf("Step 2..\n");fflush(stdout);
    if(mode == MODE_SR){
        size = sizeof(sr_edc) / sizeof(uint16_t);
        for(i = 0; i < size; i+=2) ael2005_write(file_ptr, dev, PMA_MDIO_DEVICE_ADDRESS, sr_edc[i], sr_edc[i+1]);
    }
    else if (mode == MODE_TWINAX) {
        size = sizeof(twinax_edc) / sizeof(uint16_t);
        for(i = 0; i < size; i+=2) ael2005_write(file_ptr, dev, PMA_MDIO_DEVICE_ADDRESS, twinax_edc[i], twinax_edc[i+1]);
    }

    // Step 3
    printf("Step 3..\n");fflush(stdout);
    size = sizeof(regs1) / sizeof(uint16_t);
    for(i = 0; i < size; i+=2) ael2005_write(file_ptr, dev, PMA_MDIO_DEVICE_ADDRESS, regs1[i], regs1[i+1]);
    ael2005_sleep(50);

    return 0;
}

int ael2005_initialize_1g(int file_ptr, uint32_t dev, int mode){
    int size, i;
    
    ael2005_sleep(1000);
    printf("AEL2005 Initialization Start 1G mode...\n");fflush(stdout);
    printf("Step 1 Reset..\n");fflush(stdout);
    size = sizeof(reset) / sizeof(uint16_t);
    for(i = 0; i < size; i+=2) ael2005_write(file_ptr, dev, PMA_MDIO_DEVICE_ADDRESS, reset[i], reset[i+1]);
    ael2005_sleep(200);

    printf("Step 2..\n");fflush(stdout);
    if (dev != 3) {
        size = sizeof(regs0) / sizeof(uint16_t);
        for(i = 0; i < size; i+=2) ael2005_write(file_ptr, dev, PMA_MDIO_DEVICE_ADDRESS, regs0[i], regs0[i+1]);
    }
    
    printf("Step 3..\n");fflush(stdout);
    if (dev == 3) {
        size = sizeof(regs1_1g) / sizeof(uint16_t);
        for(i = 0; i < size; i+=2) ael2005_write(file_ptr, dev, PMA_MDIO_DEVICE_ADDRESS, regs1_1g[i], regs1_1g[i+1]);
    }
    ael2005_sleep(2000);
    return 0;
}

int ael2005_write (int file_ptr, uint32_t PhyAddr, uint32_t PhyDev, uint16_t address, uint16_t data){
    XEmacLite_PhyWrite(file_ptr, PhyAddr, PhyDev, XEL_MDIO_OP_45_ADDRESS, XEL_MDIO_CLAUSE_45, address);
    XEmacLite_PhyWrite(file_ptr, PhyAddr, PhyDev, XEL_MDIO_OP_45_WRITE, XEL_MDIO_CLAUSE_45, data);
    ael2005_sleep(5);
    return 0;
}

int ael2005_read (int file_ptr, uint32_t PhyAddr, uint32_t PhyDev, uint16_t address, uint16_t *data){
    XEmacLite_PhyWrite(file_ptr, PhyAddr, PhyDev, XEL_MDIO_OP_45_ADDRESS, XEL_MDIO_CLAUSE_45, address);
    XEmacLite_PhyRead(file_ptr, PhyAddr, PhyDev, XEL_MDIO_OP_45_READ, XEL_MDIO_CLAUSE_45, data);
    ael2005_sleep(5);
    return 0;
}

int ael2005_i2c_read (int file_ptr, uint32_t PhyAddress, uint16_t dev_addr, uint16_t word_addr, uint16_t *data){
    uint16_t stat;
    int i;
    ael2005_write(file_ptr, PhyAddress, 1, AEL_I2C_CTRL, (dev_addr << 8) | (1 << 8) | word_addr);
    for (i = 0; i < 20; i++){
        ael2005_sleep(2);
        ael2005_read  (file_ptr, PhyAddress, 1, AEL_I2C_STAT, &stat);
        if ((stat & 3) == 1){
            ael2005_read(file_ptr, PhyAddress, 1, AEL_I2C_DATA, &stat);
            *data = stat >> 8;
            return 0;
        }
    }
    return -1;
}

int XEmacLite_PhyRead(int file_ptr, uint32_t PhyAddress, uint32_t RegNum, uint32_t opcode, uint32_t clause, uint16_t *PhyDataPtr) {
    uint32_t PhyAddrReg;
    //uint32_t MdioCtrlReg;
    uint32_t rd_address;
    uint32_t wr_address;
    uint32_t rd_value;

    //Verify MDIO master status.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    if ((simple_rd(file_ptr, rd_address) & XEL_MDIOCNTR_STATUS_MASK)  != 0) {
        rd_value = simple_rd(file_ptr, rd_address);
        return -1;
    }

    PhyAddrReg = ((((PhyAddress << XEL_MDIO_ADDRESS_SHIFT) & XEL_MDIO_ADDRESS_MASK) | RegNum) | opcode | clause);
    wr_address = MDIO_BASE_ADDR + XEL_MDIOADDR_OFFSET;
    simple_wr(file_ptr, wr_address, PhyAddrReg);

    //Enable MDIO and start the transfer.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    rd_value = simple_rd(file_ptr, rd_address);
    wr_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    simple_wr(file_ptr, wr_address, rd_value | XEL_MDIOCNTR_STATUS_MASK | XEL_MDIOCNTR_ENABLE_MASK);

    //Wait till the completion of transfer.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    ael2005_sleep(2);
    while ((simple_rd(file_ptr, rd_address) & XEL_MDIOCNTR_STATUS_MASK) != 0);

    //Read data from MDIO read data register.
    rd_address = MDIO_BASE_ADDR + XEL_MDIORD_OFFSET;
    *PhyDataPtr = (uint16_t)simple_rd(file_ptr, rd_address);

    //Disable the MDIO.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    rd_value = simple_rd(file_ptr, rd_address);
    wr_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    simple_wr(file_ptr, wr_address, rd_value & ~XEL_MDIOCNTR_ENABLE_MASK);

    return 0;
}

int XEmacLite_PhyWrite(int file_ptr, uint32_t PhyAddress, uint32_t RegNum, uint32_t opcode, uint32_t clause, uint16_t PhyData) {
    uint32_t PhyAddrReg;
    //uint32_t MdioCtrlReg;
    uint32_t rd_address;
    uint32_t wr_address;
    uint32_t rd_value;

    //Verify MDIO master status.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    if ((simple_rd(file_ptr, rd_address) & XEL_MDIOCNTR_STATUS_MASK)  != 0) {
        printf("XEmacLite_PhyWrite: XST_DEVICE_BUSY\n");
        return -1;
    }

    PhyAddrReg = ((((PhyAddress << XEL_MDIO_ADDRESS_SHIFT) & XEL_MDIO_ADDRESS_MASK) | RegNum) | opcode | clause);
    wr_address = MDIO_BASE_ADDR + XEL_MDIOADDR_OFFSET;
    simple_wr(file_ptr, wr_address, PhyAddrReg);

    //Write data to MDIO write data register
    wr_address = MDIO_BASE_ADDR + XEL_MDIOWR_OFFSET;
    simple_wr(file_ptr, wr_address, (uint32_t)PhyData);

    //Enable MDIO and start the transfer
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    rd_value = simple_rd(file_ptr, rd_address);
    wr_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    simple_wr(file_ptr, wr_address, rd_value | XEL_MDIOCNTR_STATUS_MASK | XEL_MDIOCNTR_ENABLE_MASK);

    //Wait till the completion of transfer.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    ael2005_sleep(2);
    while ((simple_rd(file_ptr, rd_address) & XEL_MDIOCNTR_STATUS_MASK) != 0);

    //Disable the MDIO.
    rd_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    rd_value = simple_rd(file_ptr, rd_address);
    wr_address = MDIO_BASE_ADDR + XEL_MDIOCNTR_OFFSET;
    simple_wr(file_ptr, wr_address, rd_value & ~XEL_MDIOCNTR_ENABLE_MASK);
    
    return 0;
}

uint32_t simple_rd(int file_ptr, uint32_t address){
    volatile uint64_t v;
    uint32_t rd_value;

    v = address;
    if(ioctl(file_ptr, NF10_IOCTL_CMD_READ_REG, &v) < 0){
        printf("ERROR reading AXI reg 0x%08x\n", (int)(address));
        perror("nf10 ioctl failed");
        return 0;
    }
    // upper 32bits contain the address
    // lower 32bits contain the data
    rd_value = (uint32_t)(v & 0xffffffff);
    return rd_value;
}

void simple_wr(int file_ptr, uint32_t address, uint32_t value){
    volatile uint64_t v;

    // High 32 bits are the AXI address,
    // low 32 bits are the value written to that address
    v = ((uint64_t)address << 32) | value;
    if(ioctl(file_ptr, NF10_IOCTL_CMD_WRITE_REG, &v) < 0){
        perror("nf10 ioctl failed");
    }
}

int test_status(int file_ptr){
    int i, dev;
    uint16_t pma_status, pcs_status, phy_xs_status;
    uint16_t value, port, power;
    float temperature;

    printf("**************\nCurrent Status\n**************");fflush(stdout);
    ael2005_read(file_ptr, 2, PMA_MDIO_DEVICE_ADDRESS, AEL_MICRO_CONTROLLER_CTL_ADDRESS, &pma_status);
    if (pma_status & 0x8000) {
        printf("\nAEL2005 PHY chips not programmed\n**************\n");fflush(stdout);
        return -1;
    }
    for(port = 0; port < 4; port ++){ // PHY0-3 -> C, B, A, D
        if(port == 0) dev = 2;
        if(port == 1) dev = 1;
        if(port == 2) dev = 0;
        if(port == 3) dev = 3;
        value = 0;

        printf("\nPort number: %d\n", port);
        printf("SFP module: ");fflush(stdout);
        for(i = 20; i < 36; i++){
            ael2005_i2c_read (file_ptr, dev, MODULE_DEV_ADDR, i, &value);
            printf("%c", (char)(0xff & value));fflush(stdout);
        }
        for(i = 40; i < 56; i++){
            ael2005_i2c_read (file_ptr, dev, MODULE_DEV_ADDR, i, &value);
            printf("%c", (char)(0xff & value));fflush(stdout);
        }
        printf("\n");
        // Check if we have a 10GBASE-SR cable
        ael2005_i2c_read (file_ptr, dev, MODULE_DEV_ADDR, 0x3, &value);
        if (value >> 4 == 1) printf("10GBASE-SR cable\n");

        // Temp read
        ael2005_i2c_read (file_ptr, dev, SFF_DEV_ADDR, 96, &value);
        temperature = value & 0xff;
        ael2005_i2c_read (file_ptr, dev, SFF_DEV_ADDR, 97, &value);
        value &= 0xff;
        if ((value >> 7) & 0x1) temperature += 0.5;
        if ((value >> 6) & 0x1) temperature += 0.25;
        printf("Temperature: %02.2f°\n", temperature);fflush(stdout);
        //Tx power read
        ael2005_i2c_read (file_ptr, dev, SFF_DEV_ADDR, 102, &value);
        power = (value & 0xff) << 8;
        ael2005_i2c_read (file_ptr, dev, SFF_DEV_ADDR, 103, &value);
        power |= (value & 0xff);
        printf("Tx power: %03.1f uW\n", (float)power * 0.1);fflush(stdout);
        //Rx power read
        ael2005_i2c_read (file_ptr, dev, SFF_DEV_ADDR, 104, &value);
        power = (value & 0xff) << 8;
        ael2005_i2c_read (file_ptr, dev, SFF_DEV_ADDR, 105, &value);
        power |= (value & 0xff);
        printf("Rx power: %03.1f uW\n", (float)power * 0.1);fflush(stdout);

        ael2005_read(file_ptr, dev, PMA_MDIO_DEVICE_ADDRESS, PMA_RCV_SIG_DETECTED_ADDRESS, &value);
        if (value == 0) printf("Signal detected: NO\n");
        else printf("Signal detected: YES\n");
        fflush(stdout);

        ael2005_read(file_ptr, dev, PMA_MDIO_DEVICE_ADDRESS, PMA_STATUS_REGISTER_ADDRESS, &pma_status);
        ael2005_read(file_ptr, dev, PCS_MDIO_DEVICE_ADDRESS, PCS_STATUS_REGISTER_ADDRESS, &pcs_status);
        ael2005_read(file_ptr, dev, PHY_XS_MDIO_DEVICE_ADDRESS, PHY_XS_STATUS_REGISTER_ADDRESS, &phy_xs_status);
        #ifdef DEBUG
            printf("DEBUG: pma_status: 0x%04x, pcs_status: 0x%04x, phy_xs_status: 0x%04x\n", pma_status, pcs_status, phy_xs_status);
            ael2005_read(file_ptr, dev, PMA_MDIO_DEVICE_ADDRESS, PMA_CTL_REGISTER_ADDRESS, &pma_status);
            printf("DEBUG: ctl pma: 0x%04x\n", pma_status);
        #endif
        if(((pma_status>>2) & 0x1) & ((pcs_status>>2) & 0x1) & ((phy_xs_status>>2) & 0x1)){
            printf("PHY %d OK\n", port);fflush(stdout);
        }
        else {
            printf("PHY %d error: link down\n", port);fflush(stdout);
        }

    }
    return 0;
}

void ael2005_sleep(int ms) {
    struct timespec ts_req, ts_rem;

    ts_req.tv_sec = 0;
    ts_req.tv_nsec = ms * 1000000;

    nanosleep(&ts_req, &ts_rem);
}
