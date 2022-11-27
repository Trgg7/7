module task1(input clk, input rst_n, input [7:0] start_pc, output[15:0] out);
reg out;
wire m_data;
wire [15:0] instr;
wire [7:0] next_pc;
wire [7:0] pc_add;
wire [7:0] keep_pc;

always @(*) begin
pc_add = keep_pc + 1'b1

next_pc= (clear_pc) ? start_pc : pc_add;

ram_r_addr = (sel_addr)? keep_pc : data_address;
ram_w_addr = (sel_addr)? keep_pc : data_address;


end 



always @(posedge clk) begin 

keep_pc <= (load_pc) ? next_pc : keep_pc;

instr <= (load_ir) ram_r_data : instr;

data_address <= (load_addr) ? datapath_out[7:0] : data_address;



end


idecoder U00(instr, reg_sel, opcode, ALU_op, shift_op, sximm5,  sximm8, r_addr, w_addr);
datapath U01(clk, ram_r_data, keep_pc, wb_sel, w_addr, w_en, r_addr, en_A,en_B, shift_op, sel_A,sel_B, ALU_op, en_C, en_status,sximm8, sximm5,datapath_out, Z1, N1, V1);
controller U02(clk, rst_n, start,opcode, ALU_op, shift_op,Z1, N1, V1,waiting,reg_sel, wb_sel, w_en,en_A, en_B, en_C, en_status,sel_A, sel_B, clear_pc, load_pc, load_addr, ram_w_addr, sel_addr, load_ir));
ram U03(clk, ram_w_en, ram_r_addr, ram_w_addr, datapath_out, ram_r_data);

endmodule: task1