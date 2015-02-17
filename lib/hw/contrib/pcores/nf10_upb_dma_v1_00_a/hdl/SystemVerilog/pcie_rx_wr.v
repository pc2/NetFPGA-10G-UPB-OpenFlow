//-------------------------------------------------------------
// PCIe receive module
//
// February 2012
// Mario Flajslik (mariof@stanford.edu)
// Modified by Jörg Niklas (osjsn@niklasfamily.de)
//-------------------------------------------------------------

`include "dma_defs.vh"

module pcie_rx_wr
  (
   // PCIe Transaction RX
   input logic [63:0]                trn_rd,
   input logic [7:0]                 trn_rrem_n,
   input logic                       trn_rsof_n,
   input logic                       trn_reof_n,
   input logic                       trn_rsrc_rdy_n,
   input logic                       trn_rerrfwd_n,
   input logic [6:0]                 trn_rbar_hit_n,
   
   // memory write interface
   output logic [1:0]                wr_if_select,
   output logic [3:0]                wr_mem_select,
   output logic [`MEM_ADDR_BITS-1:0] wr_addr_hi,
   output logic [31:0]               wr_data_hi,
   output logic [3:0]                wr_mask_hi,
   output logic                      wr_en_hi,
   output logic [`MEM_ADDR_BITS-1:0] wr_addr_lo,
   output logic [31:0]               wr_data_lo,
   output logic [3:0]                wr_mask_lo,
   output logic                      wr_en_lo,

   // stats
   output logic                      stat_pcie_rx_wr_cnt_inc,
   
   // misc
   input logic                       pcie_clk,
   input logic                       rst
   );

   // -----------------------------------
   // -- flop input PCIe signals
   // -----------------------------------
   logic [63:0]                      trn_rd_reg;
   logic [7:0]                       trn_rrem_n_reg;
   logic                             trn_rsof_n_reg;
   logic                             trn_reof_n_reg;
   logic                             trn_rsrc_rdy_n_reg;
   logic                             trn_rerrfwd_n_reg;
   logic [6:0]                       trn_rbar_hit_n_reg;
   always_ff @(posedge pcie_clk) begin
      trn_rd_reg <= trn_rd;
      trn_rrem_n_reg <= trn_rrem_n;
      trn_rsof_n_reg <= trn_rsof_n;
      trn_reof_n_reg <= trn_reof_n;
      trn_rsrc_rdy_n_reg <= trn_rsrc_rdy_n;
      trn_rerrfwd_n_reg  <= trn_rerrfwd_n;
      trn_rbar_hit_n_reg <= trn_rbar_hit_n;
   end

   // -----------------------------------
   // -- PCIe request processing logic
   // -----------------------------------
   localparam STATE_HEADER1 = 0;
   localparam STATE_HEADER2 = 1;
   localparam STATE_BODY    = 2;

   localparam OP_WR_ODD  = 0;
   localparam OP_WR_EVEN = 1;
   localparam OP_DROP    = 2;
   
   logic [1:0]                state,  state_nxt;
   logic [1:0]                op,     op_nxt;
   logic [63:0]               head1,  head1_nxt;
   logic [`MEM_ADDR_BITS-1:0] addr,   addr_nxt_0,   addr_nxt_2,   addr_nxt_3;
   logic [3:0]                lastBE, lastBE_nxt;

   logic [1:0]                wr_if_select_nxt_0,  wr_if_select_nxt_2,  wr_if_select_nxt_3;
   logic [3:0]                wr_mem_select_nxt_0, wr_mem_select_nxt_2, wr_mem_select_nxt_3;
   logic [`MEM_ADDR_BITS-1:0] wr_addr_hi_nxt_0,    wr_addr_hi_nxt_2,    wr_addr_hi_nxt_3;
   logic [31:0]               wr_data_hi_nxt_0,    wr_data_hi_nxt_2,    wr_data_hi_nxt_3;
   logic [3:0]                wr_mask_hi_nxt_0,    wr_mask_hi_nxt_2,    wr_mask_hi_nxt_3;
   logic                      wr_en_hi_nxt_0,      wr_en_hi_nxt_2,      wr_en_hi_nxt_3;
   logic [`MEM_ADDR_BITS-1:0] wr_addr_lo_nxt_0,    wr_addr_lo_nxt_2,    wr_addr_lo_nxt_3;
   logic [31:0]               wr_data_lo_nxt_0,    wr_data_lo_nxt_2,    wr_data_lo_nxt_3;
   logic [3:0]                wr_mask_lo_nxt_0,    wr_mask_lo_nxt_2,    wr_mask_lo_nxt_3;
   logic                      wr_en_lo_nxt_0,      wr_en_lo_nxt_2,      wr_en_lo_nxt_3;
   
   logic [1:0]                wr_mux_select; // 0 - head2-write; 1 - head2-completion; 2-body-odd; 3-body-even    


   always_comb begin

      state_nxt   = state;
      op_nxt      = op;
      head1_nxt   = head1;
      addr_nxt_0  = addr;
      addr_nxt_2  = addr;
      addr_nxt_3  = addr;
      lastBE_nxt  = lastBE;

      wr_if_select_nxt_0  = wr_if_select;
      wr_mem_select_nxt_0 = wr_mem_select;      
      wr_addr_hi_nxt_0 = 0;
      wr_data_hi_nxt_0 = 0;
      wr_mask_hi_nxt_0 = 0;
      wr_en_hi_nxt_0   = 0;
      wr_addr_lo_nxt_0 = 0;
      wr_data_lo_nxt_0 = 0;
      wr_mask_lo_nxt_0 = 0;
      wr_en_lo_nxt_0   = 0;

      wr_if_select_nxt_2  = wr_if_select;
      wr_mem_select_nxt_2 = wr_mem_select;      
      wr_addr_hi_nxt_2 = 0;
      wr_data_hi_nxt_2 = 0;
      wr_mask_hi_nxt_2 = 0;
      wr_en_hi_nxt_2   = 0;
      wr_addr_lo_nxt_2 = 0;
      wr_data_lo_nxt_2 = 0;
      wr_mask_lo_nxt_2 = 0;
      wr_en_lo_nxt_2   = 0;

      wr_if_select_nxt_3  = wr_if_select;
      wr_mem_select_nxt_3 = wr_mem_select;      
      wr_addr_hi_nxt_3 = 0;
      wr_data_hi_nxt_3 = 0;
      wr_mask_hi_nxt_3 = 0;
      wr_en_hi_nxt_3   = 0;
      wr_addr_lo_nxt_3 = 0;
      wr_data_lo_nxt_3 = 0;
      wr_mask_lo_nxt_3 = 0;
      wr_en_lo_nxt_3   = 0;

      wr_mux_select = 0;
      
      // stats
      stat_pcie_rx_wr_cnt_inc = 0;

      // --------------------
      // Processing TLPs
      // --------------------     
      if(!trn_rsrc_rdy_n_reg && trn_rerrfwd_n_reg) begin
         case(state)

           // wait for start of frame
           STATE_HEADER1: begin
              if(!trn_rsof_n_reg) begin
                 head1_nxt = trn_rd_reg;
                 state_nxt = STATE_HEADER2;
              end
           end

           // process header
           STATE_HEADER2: begin
              
              // -------------------              
              // **** 3DW WRITE ****
              // -------------------
              if((head1[62:61] == 2'b10) && (head1[60:56] == 5'b00000)) begin
                 wr_mux_select = 0;
                 // process address
                 if(!trn_rbar_hit_n_reg[0]) begin
                    wr_if_select_nxt_0  = 2'h0;
                    wr_mem_select_nxt_0 = {3'b0,  trn_rd_reg[32+`BAR0_MEM_SELECT_HI  : 32+`BAR0_MEM_SELECT_LO]};
                    addr_nxt_0          = {6'b0,  trn_rd_reg[32+`BAR0_ADDR_SELECT_HI : 32+`BAR0_ADDR_SELECT_LO]} + 4;
                    wr_addr_hi_nxt_0    = {6'b0,  trn_rd_reg[32+`BAR0_ADDR_SELECT_HI : 32+`BAR0_ADDR_SELECT_LO]};
                    wr_addr_lo_nxt_0    = {6'b0,  trn_rd_reg[32+`BAR0_ADDR_SELECT_HI : 32+`BAR0_ADDR_SELECT_LO]};         
                 end
                 else if(!trn_rbar_hit_n_reg[2]) begin
                    wr_if_select_nxt_0  = 2'h0;
                    wr_mem_select_nxt_0 = {3'b0, trn_rd_reg[32+`BAR2_MEM_SELECT_HI  : 32+`BAR2_MEM_SELECT_LO]} + 2;
                    addr_nxt_0          = trn_rd_reg[32+`BAR2_ADDR_SELECT_HI : 32+`BAR2_ADDR_SELECT_LO] + 4;
                    wr_addr_hi_nxt_0    = trn_rd_reg[32+`BAR2_ADDR_SELECT_HI : 32+`BAR2_ADDR_SELECT_LO];
                    wr_addr_lo_nxt_0    = trn_rd_reg[32+`BAR2_ADDR_SELECT_HI : 32+`BAR2_ADDR_SELECT_LO];         
                 end

                 // setup data and mask
                 wr_data_hi_nxt_0 = {trn_rd_reg[7:0], trn_rd_reg[15:8], trn_rd_reg[23:16], trn_rd_reg[31:24]};
                 wr_data_lo_nxt_0 = {trn_rd_reg[7:0], trn_rd_reg[15:8], trn_rd_reg[23:16], trn_rd_reg[31:24]};
                 wr_mask_hi_nxt_0 = {head1[3:0] & ~trn_rrem_n_reg[3:0]};
                 wr_mask_lo_nxt_0 = {head1[3:0] & ~trn_rrem_n_reg[3:0]};

                 // store operation state
                 if(trn_rd_reg[34]) begin // odd/even address
                    wr_en_hi_nxt_0 = 1;
                    op_nxt = OP_WR_EVEN;
                 end
                 else begin
                    wr_en_lo_nxt_0 = 1;
                    op_nxt = OP_WR_ODD;
                 end

                 // store last byte enable mask
                 lastBE_nxt = head1[7:4];

                 // stats
                 stat_pcie_rx_wr_cnt_inc = 1;
              end                   
               
              // -------------------
              // **** 4DW WRITE ****
              // -------------------
              else if((head1[62:61] == 2'b11) && (head1[60:56] == 5'b00000)) begin
                 wr_mux_select = 0;
                 // process address
                 if(!trn_rbar_hit_n_reg[0]) begin
                    wr_if_select_nxt_0  = 2'h0;
                    wr_mem_select_nxt_0 = {3'b0,  trn_rd_reg[`BAR0_MEM_SELECT_HI  : `BAR0_MEM_SELECT_LO]};
                    addr_nxt_0          = {6'b0,  trn_rd_reg[`BAR0_ADDR_SELECT_HI : `BAR0_ADDR_SELECT_LO]} + 4;
                 end
                 else if(!trn_rbar_hit_n_reg[2]) begin
                    wr_if_select_nxt_0  = 2'h0;
                    wr_mem_select_nxt_0 = {3'b0, trn_rd_reg[`BAR2_MEM_SELECT_HI  : `BAR2_MEM_SELECT_LO]} + 2;
                    addr_nxt_0          = trn_rd_reg[`BAR2_ADDR_SELECT_HI : `BAR2_ADDR_SELECT_LO] + 4;
                 end

                 // store operation state
                 if(trn_rd_reg[2])
                   op_nxt = OP_WR_ODD;
                 else
                   op_nxt = OP_WR_EVEN;

                 // store last byte enable mask
                 lastBE_nxt = head1[7:4];

                 // stats
                 stat_pcie_rx_wr_cnt_inc = 1;
              end

              // --------------------------
              // **** UNKNOWN ****
              // --------------------------
              else begin
                 op_nxt = OP_DROP;
              end
              
              // advance state machine
              if(!trn_reof_n_reg) begin
                 state_nxt = STATE_HEADER1;
              end
              else begin
                 state_nxt = STATE_BODY;
              end

           end
           
           // receiving body of the packet
           STATE_BODY: begin
              case(op)
                OP_WR_ODD: begin
                   wr_mux_select = 2;
                   
                   addr_nxt_2 = addr + 8;

                   wr_addr_lo_nxt_2 = addr + 4;
                   wr_data_lo_nxt_2 = {trn_rd_reg[7:0],   trn_rd_reg[15:8],  trn_rd_reg[23:16], trn_rd_reg[31:24]};
                   
                   wr_addr_hi_nxt_2 = addr;
                   wr_data_hi_nxt_2 = {trn_rd_reg[39:32], trn_rd_reg[47:40], trn_rd_reg[55:48], trn_rd_reg[63:56]};

                   if(!trn_reof_n_reg) begin // writing last word
                      if(trn_rrem_n_reg[3:0] == 4'b0000) begin
                         wr_mask_lo_nxt_2 = lastBE;
                         wr_en_lo_nxt_2   = 1;
                         
                         wr_mask_hi_nxt_2 = 4'b1111;
                         wr_en_hi_nxt_2   = 1;
                      end
                      else begin
                         wr_mask_hi_nxt_2 = lastBE;
                         wr_en_hi_nxt_2   = 1;
                      end
                   end
                   else begin // writing non-last word
                      wr_mask_lo_nxt_2 = 4'b1111;
                      wr_en_lo_nxt_2   = 1;
                      
                      wr_mask_hi_nxt_2 = 4'b1111;
                      wr_en_hi_nxt_2   = 1;
                   end                                         
                end
                
                OP_WR_EVEN: begin
                   wr_mux_select = 3;
                   
                   addr_nxt_3 = addr + 8;

                   wr_addr_lo_nxt_3 = addr;
                   wr_data_lo_nxt_3 = {trn_rd_reg[39:32], trn_rd_reg[47:40], trn_rd_reg[55:48], trn_rd_reg[63:56]};

                   wr_addr_hi_nxt_3 = addr + 4;
                   wr_data_hi_nxt_3 = {trn_rd_reg[7:0],   trn_rd_reg[15:8],  trn_rd_reg[23:16], trn_rd_reg[31:24]};                   

                   if(!trn_reof_n_reg) begin // writing last word
                      if(trn_rrem_n_reg[3:0] == 4'b0000) begin
                         wr_mask_lo_nxt_3 = 4'b1111;
                         wr_en_lo_nxt_3   = 1;

                         wr_mask_hi_nxt_3 = lastBE;
                         wr_en_hi_nxt_3   = 1;                         
                      end
                      else begin
                         wr_mask_lo_nxt_3 = lastBE;
                         wr_en_lo_nxt_3   = 1;
                      end
                   end
                   else begin // writing non-last word
                      wr_mask_lo_nxt_3 = 4'b1111;
                      wr_en_lo_nxt_3   = 1;
                      
                      wr_mask_hi_nxt_3 = 4'b1111;
                      wr_en_hi_nxt_3   = 1;
                   end                                         
                end
                OP_DROP: begin
                end
                default: begin
                end
              endcase

              // advance state machine
              if(!trn_reof_n_reg) begin
                 state_nxt = STATE_HEADER1;
              end                   
              
           end
         endcase
      end
   end

   always_ff @(posedge pcie_clk) begin
      if(rst) begin
         state    <= STATE_HEADER1;
         wr_en_lo <= 0;
         wr_en_hi <= 0;
      end
      else begin
         state    <= state_nxt;
         
         case(wr_mux_select)
           2'h0: begin
              wr_en_lo <= wr_en_lo_nxt_0;
              wr_en_hi <= wr_en_hi_nxt_0;
           end
           2'h2: begin
              wr_en_lo <= wr_en_lo_nxt_2;
              wr_en_hi <= wr_en_hi_nxt_2;
           end
           2'h3: begin
              wr_en_lo <= wr_en_lo_nxt_3;
              wr_en_hi <= wr_en_hi_nxt_3;
           end
           default: begin
              wr_en_lo <= 0;
              wr_en_hi <= 0;
           end
         endcase
      end

      op     <= op_nxt;
      head1  <= head1_nxt;
      lastBE <= lastBE_nxt;

      case(wr_mux_select)
        2'h0: begin
           addr <= addr_nxt_0;
           wr_if_select  <= wr_if_select_nxt_0;
           wr_mem_select <= wr_mem_select_nxt_0;      
           wr_addr_lo <= wr_addr_lo_nxt_0;
           wr_data_lo <= wr_data_lo_nxt_0;
           wr_mask_lo <= wr_mask_lo_nxt_0;
           wr_addr_hi <= wr_addr_hi_nxt_0;
           wr_data_hi <= wr_data_hi_nxt_0;
           wr_mask_hi <= wr_mask_hi_nxt_0;
        end
        2'h2: begin
           addr <= addr_nxt_2;
           wr_if_select  <= wr_if_select_nxt_2;
           wr_mem_select <= wr_mem_select_nxt_2;      
           wr_addr_lo <= wr_addr_lo_nxt_2;
           wr_data_lo <= wr_data_lo_nxt_2;
           wr_mask_lo <= wr_mask_lo_nxt_2;
           wr_addr_hi <= wr_addr_hi_nxt_2;
           wr_data_hi <= wr_data_hi_nxt_2;
           wr_mask_hi <= wr_mask_hi_nxt_2;
        end
        2'h3: begin
           addr <= addr_nxt_3;
           wr_if_select  <= wr_if_select_nxt_3;
           wr_mem_select <= wr_mem_select_nxt_3;      
           wr_addr_lo <= wr_addr_lo_nxt_3;
           wr_data_lo <= wr_data_lo_nxt_3;
           wr_mask_lo <= wr_mask_lo_nxt_3;
           wr_addr_hi <= wr_addr_hi_nxt_3;
           wr_data_hi <= wr_data_hi_nxt_3;
           wr_mask_hi <= wr_mask_hi_nxt_3;
        end
        default: begin
        end
      endcase

   end
endmodule
