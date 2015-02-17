/*
 * UPB ChipScope ICON core
 *
 * Copyright (c) 2014, 2015 Jörg Niklas
 * osjsn@niklasfamily.de
 *
 * This file is part of the NetFPGA 10G UPB OpenFlow Switch project.
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

`default_nettype none

module nf10_upb_chipscope_icon #(

	parameter icon_ports = 1

)(

	inout wire [35:0] control0,
	inout wire [35:0] control1,
	inout wire [35:0] control2,
	inout wire [35:0] control3,
	inout wire [35:0] control4,
	inout wire [35:0] control5

);

generate

	if (icon_ports == 1) begin
		(* box_type = "user_black_box" *)
		chipscope_icon_1_ports icon_inst (
			.CONTROL0(control0)
		);
		assign control1 = 35'b0;
		assign control2 = 35'b0;
		assign control3 = 35'b0;
		assign control4 = 35'b0;
		assign control5 = 35'b0;
	end else

	if (icon_ports == 2) begin
		(* box_type = "user_black_box" *)
		chipscope_icon_2_ports icon_inst (
			.CONTROL0(control0),
			.CONTROL1(control1)
		);
		assign control2 = 35'b0;
		assign control3 = 35'b0;
		assign control4 = 35'b0;
		assign control5 = 35'b0;
	end else

	if (icon_ports == 3) begin
		(* box_type = "user_black_box" *)
		chipscope_icon_3_ports icon_inst (
			.CONTROL0(control0),
			.CONTROL1(control1),
			.CONTROL2(control2)
		);
		assign control3 = 35'b0;
		assign control4 = 35'b0;
		assign control5 = 35'b0;
	end else

	if (icon_ports == 4) begin
		(* box_type = "user_black_box" *)
		chipscope_icon_4_ports icon_inst (
			.CONTROL0(control0),
			.CONTROL1(control1),
			.CONTROL2(control2),
			.CONTROL3(control3)
		);
		assign control4 = 35'b0;
		assign control5 = 35'b0;
	end else

	if (icon_ports == 5) begin
		(* box_type = "user_black_box" *)
		chipscope_icon_5_ports icon_inst (
			.CONTROL0(control0),
			.CONTROL1(control1),
			.CONTROL2(control2),
			.CONTROL3(control3),
			.CONTROL4(control4)
		);
		assign control5 = 35'b0;
	end

	if (icon_ports == 6) begin
		(* box_type = "user_black_box" *)
		chipscope_icon_6_ports icon_inst (
			.CONTROL0(control0),
			.CONTROL1(control1),
			.CONTROL2(control2),
			.CONTROL3(control3),
			.CONTROL4(control4),
			.CONTROL5(control5)
		);
	end

endgenerate

endmodule

module chipscope_icon_1_ports (
	inout [35:0] CONTROL0
);
endmodule

module chipscope_icon_2_ports (
	inout [35:0] CONTROL0,
	inout [35:0] CONTROL1
);
endmodule

module chipscope_icon_3_ports (
	inout [35:0] CONTROL0,
	inout [35:0] CONTROL1,
	inout [35:0] CONTROL2
);
endmodule

module chipscope_icon_4_ports (
	inout [35:0] CONTROL0,
	inout [35:0] CONTROL1,
	inout [35:0] CONTROL2,
	inout [35:0] CONTROL3
);
endmodule

module chipscope_icon_5_ports (
	inout [35:0] CONTROL0,
	inout [35:0] CONTROL1,
	inout [35:0] CONTROL2,
	inout [35:0] CONTROL3,
	inout [35:0] CONTROL4
);
endmodule

module chipscope_icon_6_ports (
	inout [35:0] CONTROL0,
	inout [35:0] CONTROL1,
	inout [35:0] CONTROL2,
	inout [35:0] CONTROL3,
	inout [35:0] CONTROL4,
	inout [35:0] CONTROL5
);
endmodule

