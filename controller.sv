`define Initial 4'b0000
`define St 4'b0001
`define single 4'b0010
`define direct 4'b0011
`define duo 4'b0100
`define duo1 4'b1100
`define single1 4'b1101
`define add 4'b0101
`define cmp 4'b0110
`define aand 4'b0111
`define mvn 4'b1000
`define movsh 4'b1001
`define write 4'b1010
`define load 4'b1011
`define halt 4'b1100
module controller(input clk, input rst_n, input start,
                  input [2:0] opcode, input [1:0] ALU_op, input [1:0] shift_op,
                  input Z, input N, input V,
                  output reg waiting,
                  output reg [1:0] reg_sel, output reg [1:0] wb_sel, output reg w_en,
                  output reg en_A, output reg en_B, output reg en_C, output reg en_status,
                  output reg sel_A, output reg sel_B);
reg rst;
wire [3:0] state;
reg [3:0] nxstate;
assign state = nxstate;
always @(posedge clk)begin

if (~rst_n)begin 
nxstate = `Initial;
end
else begin 
case(state)
`Initial : nxstate = (start)?`St:`Initial;
`St : if({opcode,ALU_op} === 5'b11010)begin nxstate = `direct;end
	else if({opcode,ALU_op} === 5'b11000||{opcode,ALU_op} === 5'b10111)begin nxstate = `single; end
	else if(opcode === 3'b111)begin nxstate = `halt; end
	else begin nxstate = `duo; end
//`duo : nxstate = `duo1;
`duo : nxstate = `single;
`direct : nxstate = `Initial;
//`single : nxstate = `single1;
`single : if({opcode,ALU_op} === 5'b10100)begin nxstate = `add; end
	else if({opcode,ALU_op} === 5'b10101)begin nxstate = `cmp; end
	else if({opcode,ALU_op} === 5'b10110)begin nxstate = `aand; end
	else if({opcode,ALU_op} === 5'b10111)begin nxstate = `mvn; end
	else if({opcode,ALU_op} === 5'b11000)begin nxstate = `movsh; end
	else begin nxstate = `Initial; end
`add : nxstate = `load;
`cmp : nxstate = `load;
`aand : nxstate = `load;
`mvn : nxstate = `load;
`movsh : nxstate = `load;
`load : nxstate = `write;
`write : nxstate = `Initial;
`halt : nxstate = `halt;

default : nxstate = `Initial;
endcase
end
end
//always @(opcode,ALU_op)begin
//nxstate = `Initial;
//end
always @(*)begin 
case(state)
`Initial : {waiting,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b100000000000;
`St : {waiting,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b000000000000;
`direct : {w_en,reg_sel,wb_sel} =5'b11010;
`single : {reg_sel,en_A,en_B} = 4'b0001;
`duo : {reg_sel,en_A,en_B} = 4'b1010;
//`duo1 : {en_A,en_B} = 2'b10;
//`single1 : {en_A,en_B} = 2'b01;
`add : {sel_A,sel_B,en_A,en_B,en_C,reg_sel} = 7'b0000101;
`cmp : begin{sel_A,sel_B,en_A,en_B,en_C,reg_sel} = 7'b0000001;
en_status = (Z|N|V)? 1'b1:1'b0;
end
`aand : {sel_A,sel_B,en_A,en_B,en_C,reg_sel} = 7'b0000101;
`mvn :{sel_A,sel_B,en_A,en_B,en_C,reg_sel} = 7'b1000101;
`movsh : {sel_A,sel_B,en_A,en_B,en_C,reg_sel} = 7'b1000101;
`write : {reg_sel,w_en,en_A,en_B} = 5'b01100;
`load : {reg_sel,en_A,en_B} = 4'b0000;
`halt : {waiting,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b000000000000;
default : {waiting,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b000000000000;
endcase
end

  // your implementation here
endmodule: controller
