`timescale 1ns / 1ps
/*
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
 */
module frame_gen_tb;

    // Inputs
    reg clk;
    reg axi_resetn;
    reg m_axis_tready;

    // Outputs
    wire [255:0] m_axis_tdata;
    wire [31:0] m_axis_tkeep;
    wire [13:0] m_axis_tuser_packet_length;
    wire [2:0] m_axis_tuser_in_port;
    wire [7:0] m_axis_tuser_out_port;
    wire [2:0] m_axis_tuser_in_vport;
    wire [7:0] m_axis_tuser_out_vport;
    wire m_axis_tvalid;
    wire m_axis_tlast;

    // Instantiate the Unit Under Test (UUT)
    frame_gen uut (
        .clk(clk), 
        .axi_resetn(axi_resetn), 
        .m_axis_tdata(m_axis_tdata), 
        .m_axis_tkeep(m_axis_tkeep), 
        .m_axis_tuser_packet_length(m_axis_tuser_packet_length), 
        .m_axis_tuser_in_port(m_axis_tuser_in_port), 
        .m_axis_tuser_out_port(m_axis_tuser_out_port), 
        .m_axis_tuser_in_vport(m_axis_tuser_in_vport), 
        .m_axis_tuser_out_vport(m_axis_tuser_out_vport), 
        .m_axis_tvalid(m_axis_tvalid), 
        .m_axis_tready(1), 
        .m_axis_tlast(m_axis_tlast)
    );

    initial begin
        // Initialize Inputs
        clk = 0;
        axi_resetn = 0;
        m_axis_tready = 0;

        // Wait 100 ns for global reset to finish
        #100;
        
        // Add stimulus here

    end
    always
        #5 clk = ~clk;   
endmodule

