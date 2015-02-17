###############################################################################
#DCI_CASCADING
#Syntax : CONFIG DCI_CASCADE = "<master> <slave1> <slave2> ..";
###############################################################################
CONFIG DCI_CASCADE = "23 27 13 25 29"; # last 3 banks are for RLDRAM-II
CONFIG DCI_CASCADE = "24 28 30 26 14"; # last 3 banks are for RLDRAM-II
CONFIG DCI_CASCADE = "5 7";
NET  "dci_masterbank_dummy_pin[*]"                   IOSTANDARD = HSTL_I_DCI;

################################################################################
# QDRII A INTERFACE ON SCHEMATIC
################################################################################

# IO STANDARDDS
NET  "qdr_a_d[*]"                              IOSTANDARD = HSTL_I;
NET  "qdr_a_q[*]"                              IOSTANDARD = HSTL_I_DCI;
NET  "qdr_a_sa[*]"                             IOSTANDARD = HSTL_I;
NET  "qdr_a_w_n"                               IOSTANDARD = HSTL_I;
NET  "qdr_a_r_n"                               IOSTANDARD = HSTL_I;
NET  "qdr_a_dll_off_n"                         IOSTANDARD = HSTL_I;
NET  "qdr_a_bw_n[*]"                           IOSTANDARD = HSTL_I;
NET  "qdr_a_cq"                                IOSTANDARD = HSTL_I_DCI | period = 3.3333333333ns high 50%;	# 300 MHz clock
NET  "qdr_a_cq_n"                              IOSTANDARD = HSTL_I_DCI | period = 3.3333333333ns high 50%;	# 300 MHz clock
NET  "qdr_a_k"                                 IOSTANDARD = HSTL_I;
NET  "qdr_a_k_n"                               IOSTANDARD = HSTL_I;
NET  "qdr_a_c"                                 IOSTANDARD = HSTL_I;
NET  "qdr_a_c_n"                               IOSTANDARD = HSTL_I;

# PINOUT
NET  "qdr_a_d[0]"                              LOC = "Y29" ;          #Bank 11
NET  "qdr_a_d[1]"                              LOC = "Y30" ;          #Bank 11
NET  "qdr_a_d[2]"                              LOC = "W31" ;          #Bank 11
NET  "qdr_a_d[3]"                              LOC = "W30" ;          #Bank 11
NET  "qdr_a_d[4]"                              LOC = "AA31" ;         #Bank 11
NET  "qdr_a_d[5]"                              LOC = "AA30" ;         #Bank 11
NET  "qdr_a_d[6]"                              LOC = "Y35" ;          #Bank 11
NET  "qdr_a_d[7]"                              LOC = "AA35" ;         #Bank 11
NET  "qdr_a_d[8]"                              LOC = "U33" ;          #Bank 11
NET  "qdr_a_d[9]"                              LOC = "AB36" ;         #Bank 11
NET  "qdr_a_d[10]"                             LOC = "AC36" ;         #Bank 11
NET  "qdr_a_d[11]"                             LOC = "V33" ;          #Bank 11
NET  "qdr_a_d[12]"                             LOC = "U32" ;          #Bank 11
NET  "qdr_a_d[13]"                             LOC = "AA34" ;         #Bank 11
NET  "qdr_a_d[14]"                             LOC = "AB34" ;         #Bank 11
NET  "qdr_a_d[15]"                             LOC = "V31" ;          #Bank 11
NET  "qdr_a_d[16]"                             LOC = "U31" ;          #Bank 11
NET  "qdr_a_d[17]"                             LOC = "AB31" ;         #Bank 11
NET  "qdr_a_d[18]"                             LOC = "AB32" ;         #Bank 11
NET  "qdr_a_d[19]"                             LOC = "W36" ;          #Bank 11
NET  "qdr_a_d[20]"                             LOC = "W35" ;          #Bank 11
NET  "qdr_a_d[21]"                             LOC = "Y37" ;          #Bank 11
NET  "qdr_a_d[22]"                             LOC = "AA37" ;         #Bank 11
NET  "qdr_a_d[23]"                             LOC = "V36" ;          #Bank 11
NET  "qdr_a_d[24]"                             LOC = "W37" ;          #Bank 11
NET  "qdr_a_d[25]"                             LOC = "AA36" ;         #Bank 11
NET  "qdr_a_d[26]"                             LOC = "W32" ;          #Bank 11
NET  "qdr_a_d[27]"                             LOC = "W33" ;          #Bank 11
NET  "qdr_a_d[28]"                             LOC = "AA32" ;         #Bank 11
NET  "qdr_a_d[29]"                             LOC = "Y32" ;          #Bank 11
NET  "qdr_a_d[30]"                             LOC = "V34" ;          #Bank 11
NET  "qdr_a_d[31]"                             LOC = "V35" ;          #Bank 11
NET  "qdr_a_d[32]"                             LOC = "Y34" ;          #Bank 11
NET  "qdr_a_d[33]"                             LOC = "Y33" ;          #Bank 11
NET  "qdr_a_d[34]"                             LOC = "T37" ;          #Bank 11
NET  "qdr_a_d[35]"                             LOC = "U37" ;          #Bank 11
# BANK 27 ASSIGNED TO HAVE ALL READ DATA TOGETHER WITH CQ, CQ_N, DCI CASCADE IS USED =>SO F31,G31 ARE IOS
NET  "qdr_a_q[0]"                              LOC = "K30" ;          #Bank 27
NET  "qdr_a_q[1]"                              LOC = "L30" ;          #Bank 27
NET  "qdr_a_q[2]"                              LOC = "H30" ;          #Bank 27
NET  "qdr_a_q[3]"                              LOC = "J30" ;          #Bank 27
NET  "qdr_a_q[4]"                              LOC = "L32" ;          #Bank 27
NET  "qdr_a_q[5]"                              LOC = "M33" ;          #Bank 27
NET  "qdr_a_q[6]"                              LOC = "F31" ;          #Bank 27
NET  "qdr_a_q[7]"                              LOC = "G31" ;          #Bank 27
NET  "qdr_a_q[8]"                              LOC = "M32" ;          #Bank 27
NET  "qdr_a_q[9]"                              LOC = "M31" ;          #Bank 27
NET  "qdr_a_q[10]"                             LOC = "F30" ;          #Bank 27
NET  "qdr_a_q[11]"                             LOC = "N30" ;          #Bank 27
NET  "qdr_a_q[12]"                             LOC = "N31" ;          #Bank 27
NET  "qdr_a_q[13]"                             LOC = "F29" ;          #Bank 27
NET  "qdr_a_q[14]"                             LOC = "E29" ;          #Bank 27
NET  "qdr_a_q[15]"                             LOC = "P32" ;          #Bank 27
NET  "qdr_a_q[16]"                             LOC = "P31" ;          #Bank 27
NET  "qdr_a_q[17]"                             LOC = "H29" ;          #Bank 27
NET  "qdr_a_q[18]"                             LOC = "G29" ;          #Bank 27
NET  "qdr_a_q[19]"                             LOC = "R30" ;          #Bank 27
NET  "qdr_a_q[20]"                             LOC = "P30" ;          #Bank 27
NET  "qdr_a_q[21]"                             LOC = "H33" ;          #Bank 27
NET  "qdr_a_q[22]"                             LOC = "J32" ;          #Bank 27
NET  "qdr_a_q[23]"                             LOC = "G36" ;          #Bank 27
NET  "qdr_a_q[24]"                             LOC = "F35" ;          #Bank 27
NET  "qdr_a_q[25]"                             LOC = "G34" ;          #Bank 27
NET  "qdr_a_q[26]"                             LOC = "F34" ;          #Bank 27
NET  "qdr_a_q[27]"                             LOC = "E36" ;          #Bank 27
NET  "qdr_a_q[28]"                             LOC = "F32" ;          #Bank 27
NET  "qdr_a_q[29]"                             LOC = "G32" ;          #Bank 27
NET  "qdr_a_q[30]"                             LOC = "F37" ;          #Bank 27
NET  "qdr_a_q[31]"                             LOC = "E37" ;          #Bank 27
NET  "qdr_a_q[32]"                             LOC = "J31" ;          #Bank 27
NET  "qdr_a_q[33]"                             LOC = "H31" ;          #Bank 27
NET  "qdr_a_q[34]"                             LOC = "K32" ;          #Bank 27
NET  "qdr_a_cq"                                LOC = "L31" ;          #Bank 27
NET  "qdr_a_q[35]"                             LOC = "L29" ;          #Bank 27
NET  "qdr_a_cq_n"                              LOC = "K29" ;          #Bank 27
# BANK 23 NEW ASSIGNED TO HOLD ALL ADR, CTL, CLKS
NET  "qdr_a_sa[0]"                             LOC = "H34" ;          #Bank 23
NET  "qdr_a_sa[1]"                             LOC = "J33" ;          #Bank 23
NET  "qdr_a_sa[2]"                             LOC = "M36" ;          #Bank 23
NET  "qdr_a_sa[3]"                             LOC = "N36" ;          #Bank 23
NET  "qdr_a_sa[4]"                             LOC = "N33" ;          #Bank 23
NET  "qdr_a_sa[5]"                             LOC = "P33" ;          #Bank 23
NET  "qdr_a_sa[6]"                             LOC = "R32" ;          #Bank 23
NET  "qdr_a_sa[7]"                             LOC = "R33" ;          #Bank 23
NET  "qdr_a_sa[8]"                             LOC = "N34" ;          #Bank 23
NET  "qdr_a_sa[9]"                             LOC = "J35" ;          #Bank 23
NET  "qdr_a_sa[10]"                            LOC = "J36" ;          #Bank 23
NET  "qdr_a_sa[11]"                            LOC = "L37" ;          #Bank 23
NET  "qdr_a_sa[12]"                            LOC = "M37" ;          #Bank 23
NET  "qdr_a_sa[13]"                            LOC = "K38" ;          #Bank 23
NET  "qdr_a_sa[14]"                            LOC = "K37" ;          #Bank 23
NET  "qdr_a_sa[15]"                            LOC = "T32" ;          #Bank 23
NET  "qdr_a_sa[16]"                            LOC = "T31" ;          #Bank 23
NET  "qdr_a_sa[17]"                            LOC = "K35" ;          #Bank 23
NET  "qdr_a_sa[18]"                            LOC = "L34" ;          #Bank 23
NET  "qdr_a_bw_n[0]"                           LOC = "P37" ;          #Bank 23
NET  "qdr_a_w_n"                               LOC = "P38" ;          #Bank 23
NET  "qdr_a_bw_n[1]"                           LOC = "R38" ;          #Bank 23
NET  "qdr_a_r_n"                               LOC = "G37" ;          #Bank 23
NET  "qdr_a_bw_n[2]"                           LOC = "P36" ;          #Bank 23
NET  "qdr_a_k_n"                               LOC = "K33" ;          #Bank 23
NET  "qdr_a_k"                                 LOC = "K34" ;          #Bank 23
NET  "qdr_a_c_n"                               LOC = "N35" ;          #Bank 23
NET  "qdr_a_c"                                 LOC = "P35" ;          #Bank 23
NET  "qdr_a_bw_n[3]"                           LOC = "J38" ;          #Bank 23
NET  "qdr_a_dll_off_n"                         LOC = "J37" ;          #Bank 23 # pull-up on pcb
# SINCE NO DATA READ SIGNAL ON BANK23, DUMMY MASTERSEL PIN MUST BE ALLOCATED
NET  "dci_masterbank_dummy_pin[0]"                   LOC = "R35" ;          #Bank 23

########################################################################
# QDRII B INTERFACE ON SCHEMATIC
########################################################################

# IO STANDARDDS
NET  "qdr_b_d[*]"                              IOSTANDARD = HSTL_I;
NET  "qdr_b_q[*]"                              IOSTANDARD = HSTL_I_DCI;
NET  "qdr_b_sa[*]"                             IOSTANDARD = HSTL_I;
NET  "qdr_b_w_n"                               IOSTANDARD = HSTL_I;
NET  "qdr_b_r_n"                               IOSTANDARD = HSTL_I;
NET  "qdr_b_dll_off_n"                         IOSTANDARD = HSTL_I;
NET  "qdr_b_bw_n[*]"                           IOSTANDARD = HSTL_I;
NET  "qdr_b_cq"                                IOSTANDARD = HSTL_I_DCI | period = 3.3333333333ns high 50%;	# 300 MHz clock
NET  "qdr_b_cq_n"                              IOSTANDARD = HSTL_I_DCI | period = 3.3333333333ns high 50%;	# 300 MHz clock
NET  "qdr_b_k"                                 IOSTANDARD = HSTL_I;
NET  "qdr_b_k_n"                               IOSTANDARD = HSTL_I;
NET  "qdr_b_c"                                 IOSTANDARD = HSTL_I;
NET  "qdr_b_c_n"                               IOSTANDARD = HSTL_I;

# PINOUT

# BANK 3 AND BANK 1 NEW ASSIGNED TO HOLD ALL WRITE DATA
NET  "qdr_b_d[0]"                              LOC = "M19" ;          #Bank 3
NET  "qdr_b_d[1]"                              LOC = "L19" ;          #Bank 3
NET  "qdr_b_d[2]"                              LOC = "M24" ;          #Bank 3
NET  "qdr_b_d[3]"                              LOC = "N24" ;          #Bank 3
NET  "qdr_b_d[4]"                              LOC = "J16" ;          #Bank 3
NET  "qdr_b_d[5]"                              LOC = "J15" ;          #Bank 3
NET  "qdr_b_d[6]"                              LOC = "L25" ;          #Bank 3
NET  "qdr_b_d[7]"                              LOC = "M26" ;          #Bank 3
NET  "qdr_b_d[8]"                              LOC = "K15" ;          #Bank 3
NET  "qdr_b_d[9]"                              LOC = "L15" ;          #Bank 3
NET  "qdr_b_d[10]"                             LOC = "L27" ;          #Bank 3
NET  "qdr_b_d[11]"                             LOC = "L26" ;          #Bank 3
NET  "qdr_b_d[12]"                             LOC = "M18" ;          #Bank 3
NET  "qdr_b_d[13]"                             LOC = "M17" ;          #Bank 3
NET  "qdr_b_d[14]"                             LOC = "K28" ;          #Bank 3
NET  "qdr_b_d[15]"                             LOC = "K27" ;          #Bank 3
NET  "qdr_b_d[16]"                             LOC = "L17" ;          #Bank 3
NET  "qdr_b_d[17]"                             LOC = "L16" ;          #Bank 3
NET  "qdr_b_d[18]"                             LOC = "M27" ;          #Bank 3
NET  "qdr_b_d[19]"                             LOC = "M28" ;          #Bank 3
NET  "qdr_b_d[20]"                             LOC = "P25" ;          #Bank 1
NET  "qdr_b_d[21]"                             LOC = "P26" ;          #Bank 1
NET  "qdr_b_d[22]"                             LOC = "P18" ;          #Bank 1
NET  "qdr_b_d[23]"                             LOC = "N19" ;          #Bank 1
NET  "qdr_b_d[24]"                             LOC = "N25" ;          #Bank 1
NET  "qdr_b_d[25]"                             LOC = "N26" ;          #Bank 1
NET  "qdr_b_d[26]"                             LOC = "P17" ;          #Bank 1
NET  "qdr_b_d[27]"                             LOC = "N18" ;          #Bank 1
NET  "qdr_b_d[28]"                             LOC = "N28" ;          #Bank 1
NET  "qdr_b_d[29]"                             LOC = "M14" ;          #Bank 1
NET  "qdr_b_d[30]"                             LOC = "L14" ;          #Bank 1
NET  "qdr_b_d[31]"                             LOC = "N29" ;          #Bank 1
NET  "qdr_b_d[32]"                             LOC = "M29" ;          #Bank 1
NET  "qdr_b_d[33]"                             LOC = "N16" ;          #Bank 1
NET  "qdr_b_d[34]"                             LOC = "M16" ;          #Bank 1
NET  "qdr_b_d[35]"                             LOC = "R27" ;          #Bank 1
# BANK 7 IS NEW ASSIGNED TO KEEP ALL DQS IN ONE BANK TOGETHER WITH CQ/CQ_N, DCI CASCADE => C20,D20 ARE IOS
NET  "qdr_b_q[0]"                              LOC = "J21" ;          #Bank 7
NET  "qdr_b_q[1]"                              LOC = "J22" ;          #Bank 7
NET  "qdr_b_q[2]"                              LOC = "H21" ;          #Bank 7
NET  "qdr_b_q[3]"                              LOC = "J20" ;          #Bank 7
NET  "qdr_b_q[4]"                              LOC = "A22" ;          #Bank 7
NET  "qdr_b_q[5]"                              LOC = "B21" ;          #Bank 7
NET  "qdr_b_q[6]"                              LOC = "C20" ;          #Bank 7
NET  "qdr_b_q[7]"                              LOC = "D20" ;          #Bank 7
NET  "qdr_b_q[8]"                              LOC = "F24" ;          #Bank 7
NET  "qdr_b_q[9]"                              LOC = "G23" ;          #Bank 7
NET  "qdr_b_q[10]"                             LOC = "A20" ;          #Bank 7
NET  "qdr_b_q[11]"                             LOC = "N23" ;          #Bank 7
NET  "qdr_b_q[12]"                             LOC = "M23" ;          #Bank 7
NET  "qdr_b_q[13]"                             LOC = "K20" ;          #Bank 7
NET  "qdr_b_q[14]"                             LOC = "L20" ;          #Bank 7
NET  "qdr_b_q[15]"                             LOC = "E23" ;          #Bank 7
NET  "qdr_b_q[16]"                             LOC = "E24" ;          #Bank 7
NET  "qdr_b_q[17]"                             LOC = "P22" ;          #Bank 7
NET  "qdr_b_q[18]"                             LOC = "P21" ;          #Bank 7
NET  "qdr_b_q[19]"                             LOC = "E25" ;          #Bank 7
NET  "qdr_b_q[20]"                             LOC = "F25" ;          #Bank 7
NET  "qdr_b_q[21]"                             LOC = "M21" ;          #Bank 7
NET  "qdr_b_q[22]"                             LOC = "N21" ;          #Bank 7
NET  "qdr_b_q[23]"                             LOC = "L22" ;          #Bank 7
NET  "qdr_b_q[24]"                             LOC = "M22" ;          #Bank 7
NET  "qdr_b_q[25]"                             LOC = "E22" ;          #Bank 7
NET  "qdr_b_q[26]"                             LOC = "D23" ;          #Bank 7
NET  "qdr_b_q[27]"                             LOC = "D22" ;          #Bank 7
NET  "qdr_b_q[28]"                             LOC = "G22" ;          #Bank 7
NET  "qdr_b_q[29]"                             LOC = "F22" ;          #Bank 7
NET  "qdr_b_q[30]"                             LOC = "D21" ;          #Bank 7
NET  "qdr_b_q[31]"                             LOC = "C21" ;          #Bank 7
NET  "qdr_b_q[32]"                             LOC = "L21" ;          #Bank 7
NET  "qdr_b_q[33]"                             LOC = "K22" ;          #Bank 7
NET  "qdr_b_q[34]"                             LOC = "B23" ;          #Bank 7
NET  "qdr_b_cq"                                LOC = "B22" ;          #Bank 7
NET  "qdr_b_q[35]"                             LOC = "G21" ;          #Bank 7
NET  "qdr_b_cq_n"                              LOC = "F21" ;          #Bank 7
#BANK 5 NEW ASSIGNED TO HOLD ALL ADR/ CTL / CLKS
NET  "qdr_b_sa[0]"                             LOC = "J23" ;          #Bank 5
NET  "qdr_b_sa[1]"                             LOC = "K23" ;          #Bank 5
NET  "qdr_b_sa[2]"                             LOC = "K19" ;          #Bank 5
NET  "qdr_b_sa[3]"                             LOC = "K18" ;          #Bank 5
NET  "qdr_b_sa[4]"                             LOC = "J25" ;          #Bank 5
NET  "qdr_b_sa[5]"                             LOC = "K25" ;          #Bank 5
NET  "qdr_b_sa[6]"                             LOC = "F27" ;          #Bank 5
NET  "qdr_b_sa[7]"                             LOC = "G27" ;          #Bank 5
NET  "qdr_b_sa[8]"                             LOC = "E19" ;          #Bank 5
NET  "qdr_b_sa[9]"                             LOC = "H26" ;          #Bank 5
NET  "qdr_b_sa[10]"                            LOC = "J26" ;          #Bank 5
NET  "qdr_b_sa[11]"                            LOC = "F16" ;          #Bank 5
NET  "qdr_b_sa[12]"                            LOC = "F17" ;          #Bank 5
NET  "qdr_b_sa[13]"                            LOC = "G28" ;          #Bank 5
NET  "qdr_b_sa[14]"                            LOC = "H28" ;          #Bank 5
NET  "qdr_b_sa[15]"                            LOC = "G16" ;          #Bank 5
NET  "qdr_b_sa[16]"                            LOC = "H16" ;          #Bank 5
NET  "qdr_b_sa[17]"                            LOC = "J27" ;          #Bank 5
NET  "qdr_b_sa[18]"                            LOC = "J28" ;          #Bank 5
NET  "qdr_b_bw_n[0]"                           LOC = "F20" ;          #Bank 5
NET  "qdr_b_w_n"                               LOC = "E20" ;          #Bank 5
NET  "qdr_b_bw_n[1]"                           LOC = "L24" ;          #Bank 5
NET  "qdr_b_r_n"                               LOC = "K24" ;          #Bank 5
NET  "qdr_b_bw_n[2]"                           LOC = "H19" ;          #Bank 5
NET  "qdr_b_k_n"                               LOC = "H23" ;          #Bank 5
NET  "qdr_b_k"                                 LOC = "G19" ;          #Bank 5
NET  "qdr_b_c_n"                               LOC = "F19" ;          #Bank 5
NET  "qdr_b_c"                                 LOC = "H25" ;          #Bank 5
NET  "qdr_b_bw_n[3]"                           LOC = "G24" ;          #Bank 5
NET  "qdr_b_dll_off_n"                         LOC = "G17" ;          #Bank 5 #HWIRED ON BOARD
# SINCE NO DATA READ SIGNAL ON BANK23, DUMMY MASTERSEL PIN MUST BE ALLOCATED
NET  "dci_masterbank_dummy_pin[1]"                   LOC = "F26" ;          #Bank 5
