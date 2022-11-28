`define Initial 5'b00000
`define St 5'b00001
`define single 5'b00010
`define direct 5'b00011
`define duo 5'b00100 
`define addand 5'b00101
`define cmp 5'b00110
`define movsh 5'b01001
`define write 5'b01010
`define ldrstr 5'b00111
`define load 5'b01000
`define ldr 5'b01100
`define str 5'b01101
`define str1 5'b01110
`define str2 5'b01111
`define halt 5'b01011
`define initial1 5'b01100
`define initial2 5'b01101
`define transit 5'b01110

module controller(input clk, input rst_n, input [2:0] opcode, input [1:0] ALU_op, input [1:0] shift_op,
                  input Z, input N, input V,
                  output wire waiting,
                  output reg [1:0] reg_sel, output reg [1:0] wb_sel, output reg w_en,
                  output reg en_A, output reg en_B, output reg en_C, output reg en_status,
                  output reg sel_A, output reg sel_B ,output reg clear_pc, output reg load_pc, output reg load_addr, output reg ram_w_addr, output reg sel_addr, output reg load_ir, output reg ram_w_en);
reg waitreg;
reg rst;
wire [4:0] state;
reg [4:0] nxstate;
assign state = nxstate;
assign waiting = waitreg;
always @(posedge clk)begin

if (~rst_n)begin 
nxstate = `Initial;
end
else begin 
case(state)
`Initial : nxstate = `initial1;
//program counter

`initial1 : nxstate = `initial2;
//ram read

`transit : nxstate = `initial2;
//ram read

`initial2 : nxstate = `St;
//instruction register

`St : if({opcode,ALU_op} === 5'b11010)begin nxstate = `direct;end
	else if({opcode,ALU_op} === 5'b11000||{opcode,ALU_op} === 5'b10111)begin nxstate = `single; end
else if (opcode === 3'b111) begin nxstate = `halt; end
	else begin nxstate = `duo; end
//`duo : nxstate = `duo1;
`duo : nxstate = `single;
`direct : nxstate = `Initial;
//`single : nxstate = `single1;
`single : if({opcode,ALU_op} === 5'b10100)begin nxstate = `addand; end
	else if({opcode,ALU_op} === 5'b10101)begin nxstate = `cmp; end
	else if({opcode,ALU_op} === 5'b10110)begin nxstate = `addand; end
	else if({opcode,ALU_op} === 5'b10111)begin nxstate = `movsh; end
	else if({opcode,ALU_op} === 5'b11000)begin nxstate = `movsh; end
	else if({opcode,ALU_op} === 5'b01100)begin nxstate = `ldrstr;end
	else if({opcode,ALU_op} == 5'b10000) begin nxstate = `ldrstr; end
	else begin nxstate = `Initial; end
`ldrstr : nxstate = `load;
`load : begin
                     if({opcode, ALU_op} == 5'b01100) nxstate = `ldr;
                     else if({opcode, ALU_op} == 5'b10000) nxstate = `str;
                   end
`str : nxstate = `str1;
`str1: nxstate = `St;
`ldr : nxstate = `write;
`addand : nxstate = `write;
`cmp : nxstate = `Initial;
`movsh : nxstate = `write;

`write : nxstate = `transit;

`halt : nxstate = `halt;
default : nxstate = `Initial;
endcase
end
end

always @(*)begin 
case(state)
`Initial : begin
 {waitreg,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b100000000000;
clear_pc = 1'b1;
load_pc = 1'b1;
//load_addr =
//ram_w_addr =
sel_addr = 1'b1;
load_ir = 1'b0;
end

`initial1 : begin
 {waitreg,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b100000000000;
clear_pc = 1'b1;
load_pc = 1'b0;
//load_addr =
//ram_w_addr =
//sel_addr =
load_ir = 1'b1;
end

`initial2 : begin
 {waitreg,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b100000000000;
clear_pc = 1'b1;
load_pc = 1'b1;
ram_w_en = 1'b1;
//load_addr =
//ram_w_addr =
//sel_addr =
load_ir = 1'b1;
end


`halt : begin
 {waitreg,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b100000000000;
//clear_pc = 1'b1;
load_pc = 1'b0;
//load_addr =
//ram_w_addr =
//sel_addr =
load_ir = 1'b0;
end

`transit: begin
 {waitreg,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b100000000000;
clear_pc = 1'b0;
load_pc = 1'b0;
//load_addr =
//ram_w_addr =
//sel_addr =
load_ir = 1'b1;
end


`St : begin {waitreg,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b000000000000;
//clear_pc = 1'b1;
load_pc = 1'b0;
//load_addr =
//ram_w_addr =
//sel_addr =
ram_w_en = 1'b0;
load_ir = 1'b0;
end

`direct : {w_en,reg_sel,wb_sel} =5'b11010;
`single : {reg_sel,en_A,en_B} = 4'b0001;
`duo : {reg_sel,en_A,en_B} = 4'b1010;
//`duo1 : {en_A,en_B} = 2'b10;
//`single1 : {en_A,en_B} = 2'b01;
`addand : {sel_A,sel_B,en_A,en_B,en_C,reg_sel} = 7'b0000101;
`cmp : begin{sel_A,sel_B,en_A,en_B,en_C,reg_sel} = 7'b0000001;
en_status = (Z|N|V)? 1'b1:1'b0;
end
`ldrstr : {sel_A,sel_B,en_A,en_B} = 4'b0101;
`movsh : {sel_A,sel_B,en_A,en_B,en_C,reg_sel} = 7'b1000101;

`write : begin {reg_sel,w_en,en_A,en_B} = 5'b01100;
wb_sel = ({opcode,ALU_op} == 5'b01100)? 2'b11:2'b00;
clear_pc = 1'b1;
load_pc = 1'b1;
//load_addr =
//ram_w_addr =
//sel_addr =
load_ir = 1'b0;
end


`load : {load_addr,sel_addr,en_C} = 3'b100;
`ldr : {load_addr} = 1'b0;
`str : {load_addr,en_B,reg_sel} = 4'b0101;
`str1 : {sel_A,sel_B,en_C,en_B} = 4'b1010;
default : begin {waitreg,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b000000000000;
end

endcase
end

  // your implementation here
endmodule: controller

