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
`define halt 5'b11011
`define initial1 5'b11100
`define initial2 5'b11101
`define transit 5'b11110
`define St1 5'b10000
`define strwrite 5'b10001
module task3(input clk, input rst_n, input [7:0] start_pc, output[15:0] out);
wire m_data;
reg [15:0] instr;
reg [7:0] next_pc;
reg [7:0] pc_add;
//wire [7:0] keep_pc;



//reg instr;
reg [1:0] reg_sel;
wire [2:0] opcode;
wire[1:0] ALU_op;
wire [1:0] shift_op;
wire[15:0] sximm5;
wire[15:0] sximm8;
wire[2:0] r_addr;
wire[2:0] w_addr;
wire [15:0] ram_r_data;
reg [7:0] keep_pc;
reg [1:0] wb_sel;


//reg w_en;
reg en_A;
reg en_B;
reg en_C;
reg en_status;
reg sel_A;
reg sel_B;

reg [7:0] data_address;


reg w_en;
//wire r_addr;
wire Z1;
wire N1;
wire V1;

wire waiting;
wire clear_pc;
wire load_pc;
wire load_addr;
reg [7:0] ram_w_addr;
wire sel_addr;
wire load_ir;
wire ram_w_en;
reg [7:0] ram_r_addr;
wire [15:0] datapath_out;
//wire ram_r_data;

assign out = datapath_out;



always @(*) begin
pc_add = keep_pc + 1'b1;
next_pc = (clear_pc) ? start_pc : pc_add;
ram_r_addr = (sel_addr)? keep_pc : data_address;
ram_w_addr = (sel_addr)? keep_pc : data_address;
//ram_w_addr = ram_r_addr;
end 



always @(posedge clk) begin
keep_pc <= (load_pc) ? next_pc : keep_pc;
instr <= (load_ir) ? ram_r_data : instr;
data_address <= (load_addr) ? datapath_out[7:0] : data_address;
end


idecoder U00(instr, reg_sel, opcode, ALU_op, shift_op, sximm5,  sximm8, r_addr, w_addr);
datapath U01(clk, ram_r_data, keep_pc, wb_sel, w_addr, w_en, r_addr, en_A, en_B, shift_op, sel_A, sel_B, ALU_op, en_C, en_status,sximm8, sximm5,datapath_out, Z1, N1, V1,opcode);

controller U02(clk, rst_n,opcode, ALU_op, shift_op,Z1, N1, V1,waiting,reg_sel, wb_sel, w_en,en_A, en_B, en_C, en_status,sel_A, sel_B, clear_pc, load_pc, load_addr, ram_w_addr, sel_addr, load_ir, ram_w_en);

ram U03(clk, ram_w_en, ram_r_addr, ram_r_addr, datapath_out, ram_r_data);

endmodule: task3


module controller(input clk, input rst_n, input [2:0] opcode, input [1:0] ALU_op, input [1:0] shift_op,
                  input Z, input N, input V,
                  output wire waiting,
                  output reg [1:0] reg_sel, output reg [1:0] wb_sel, output reg w_en,
                  output reg en_A, output reg en_B, output reg en_C, output reg en_status,
                  output reg sel_A, output reg sel_B ,output reg clear_pc, output reg load_pc, output reg load_addr, output reg [7:0] ram_w_addr, output reg sel_addr, output reg load_ir, output reg ram_w_en);
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

`initial1 : nxstate = `St;
//ram read

`transit: nxstate = `initial2;
//ram read

`initial2 : nxstate = `initial1;
`St: nxstate = 	`St1;
//instruction register

`St1 : if({opcode,ALU_op} === 5'b11010)begin nxstate = `direct;end
	else if({opcode,ALU_op} === 5'b11000||{opcode,ALU_op} === 5'b10111)begin nxstate = `single; end
	else if (opcode === 3'b111) begin nxstate = `halt; end
//	else if ({opcode,ALU_op} === 5'b01100 || {opcode,ALU_op}==5'b10000)begin nxstate = `loadduo;
	else begin nxstate = `duo; end
//`duo : nxstate = `duo1;
`duo : nxstate = `single;
`direct : nxstate = `initial2;
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
`str1: nxstate = `strwrite;
`strwrite : nxstate = `initial2;
`ldr : nxstate = `write;
`addand : nxstate = `write;
`cmp : nxstate = `initial2;
`movsh : nxstate = `write;

`write : nxstate = `initial2;

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
load_pc = 1'b0;
//load_addr =
//ram_w_addr =
//sel_addr =
sel_addr=1'b1;
load_ir = 1'b0;
end

`initial2 : begin
 {waitreg,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b000000000000;
clear_pc = 1'b0;
load_pc = 1'b1;
//load_addr =
//ram_w_addr =
//sel_addr =
load_ir = 1'b0;
sel_addr = 1'b1;
ram_w_en = 1'b0;
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
load_pc = 1'b1;
//load_addr =
//ram_w_addr =
//sel_addr =
sel_addr = 1'b1;
load_ir = 1'b1;
end


`St : begin {waitreg,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b000000000000;
//clear_pc = 1'b1;
load_pc = 1'b0;
//load_addr =
//ram_w_addr =
//sel_addr =
ram_w_en = 1'b0;
load_ir = 1'b1;
end
`St1 : load_ir = 1'b0;
`direct : {w_en,reg_sel,wb_sel} =5'b11010;
`single : {reg_sel,en_A,en_B} = 4'b0001;
`duo : {reg_sel,en_A,en_B} = 4'b1010;
//`duo1 : {en_A,en_B} = 2'b10;
//`single1 : {en_A,en_B} = 2'b01;
`addand : {sel_A,sel_B,en_A,en_B,en_C,reg_sel} = 7'b0000101;
`cmp : begin{sel_A,sel_B,en_A,en_B,en_C,reg_sel} = 7'b0000001;
en_status = (Z|N|V)? 1'b1:1'b0;
end
`ldrstr : {sel_A,sel_B,en_A,en_B,en_C} = 5'b01011;
`movsh : {sel_A,sel_B,en_A,en_B,en_C,reg_sel} = 7'b1000101;

`write : begin {reg_sel,w_en,en_A,en_B} = 5'b01100;
wb_sel = ({opcode,ALU_op} == 5'b01100)? 2'b11:2'b00;
//load_addr =
//ram_w_addr =
//sel_addr =
load_ir = 1'b0;
end


`load : {load_addr,sel_addr,en_C} = 3'b100;
`ldr : {load_addr,wb_sel} = 3'b011;
`str : {load_addr,sel_A,en_B,reg_sel} = 5'b00101;
`str1 : {sel_A,sel_B,en_C,en_B} = 4'b1010;
`strwrite : ram_w_en = 1'b1;
default : begin {waitreg,reg_sel,wb_sel,w_en,en_A,en_B,en_C,en_status,sel_A,sel_B} = 12'b000000000000;
end

endcase
end

  // your implementation here
endmodule: controller



module idecoder(input [15:0] ir, input [1:0] reg_sel,
                output [2:0] opcode, output [1:0] ALU_op, output [1:0] shift_op,
		output [15:0] sximm5, output [15:0] sximm8,
                output [2:0] r_addr, output [2:0] w_addr);
reg[2:0] wraddr;
assign opcode =  ir [15:13];
assign ALU_op = ir[12:11];
assign sximm5 = {{11{ir[4]}},ir[4:0]};
assign sximm8 = {{8{ir[7]}},ir[7:0]};
assign shift_op = ir[4:3];
always @(*)begin
case(reg_sel)
2'b00: wraddr = ir[2:0];
2'b01: wraddr = ir[7:5];
2'b10: wraddr = ir[10:8];
default: wraddr = 3'bxxx;
endcase
end
assign r_addr = wraddr;
assign w_addr = wraddr;
  // your implementation here
endmodule: idecoder




module datapath(input clk, input [15:0] mdata, input [7:0] pc, input [1:0] wb_sel,
                input [2:0] w_addr, input w_en, input [2:0] r_addr, input en_A,
                input en_B, input [1:0] shift_op, input sel_A, input sel_B,
                input [1:0] ALU_op, input en_C, input en_status,
		input [15:0] sximm8, input [15:0] sximm5,
                output [15:0] datapath_out, output Z_out, output N_out, output V_out,input [2:0] opcode);

wire [15:0] val_A;
wire [15:0] val_B;
reg [15:0] w_data;
wire [15:0] wwdata;
reg [15:0] a_out;
reg [15:0] b_out;
reg [15:0] r_data;
reg [15:0] outpt;
reg [15:0] shift_out;
wire [15:0] ALU_out;
assign datapath_out = outpt;
assign wwdata = w_data;

always @(*)begin
case (wb_sel)
2'b00:w_data = outpt;
2'b01:w_data = {8'b0,pc};
2'b10:w_data = sximm8;
2'b11:w_data = mdata;
endcase
end
regfile U00(wwdata, w_addr, w_en, r_addr, clk, r_data);
always @(posedge clk)begin
a_out = (en_A)? r_data:a_out;
end
always @(posedge clk)begin
b_out = (en_B)? r_data:b_out;
end

shifter U01(b_out,opcode,ALU_op, shift_op, shift_out);
assign val_A = (sel_A)? 16'b0 : a_out;
assign val_B = (sel_B)? sximm5 : shift_out;
ALU U02(val_A, val_B, ALU_op, ALU_out, Z_out,N_out,V_out);

always @(posedge clk)begin
outpt = (en_C)? ALU_out:outpt;
end
  // your implementation here
endmodule: datapath

module shifter(input [15:0] shift_in,input[2:0] opcode,input[1:0]ALU_op, input [1:0] shift_op, output reg [15:0] shift_out);
reg [15:0] shift_reg;
assign shift_out = shift_reg;
always @(*)begin
if({opcode,ALU_op}===5'b01100||{opcode,ALU_op}===5'b10000) shift_reg = shift_in;
else case(shift_op)
2'b00 : shift_reg = shift_in;
2'b01 : shift_reg = {shift_in[14:0],1'b0};
2'b10 : shift_reg = {1'b0 , shift_in[15:1]};
2'b11 : shift_reg = {shift_in[15] , shift_in[15:1]};
default : shift_reg = 16'bxxxxxxxxxxxxxxxx;
endcase
end
endmodule: shifter


module regfile(input [15:0] w_data, input [2:0] w_addr, input w_en, input [2:0] r_addr, input clk, output [15:0] r_data);
  reg [15:0] m[0:7];
  assign r_data = m[r_addr];
  always_ff @(posedge clk) if (w_en) m[w_addr] <= w_data;
endmodule: regfile



module ALU(input [15:0] val_A, input [15:0] val_B, input [1:0] ALU_op, output [15:0] ALU_out, output Z,output N, output V);
  reg [15:0] ALU_reg;
reg [15:0] vreg;
assign ALU_out = ALU_reg;
assign Z = ~(|ALU_out);
assign N = ALU_reg[15];
assign V = vreg[15];
always @(*)begin
case(ALU_op)
2'b00 : ALU_reg = val_A+val_B;
2'b01 : ALU_reg = val_A-val_B;
2'b10 : ALU_reg = val_A & val_B;
2'b11 : ALU_reg = ~val_B;
default: ALU_reg = 16'bxxxxxxxxxxxxxxxx;
endcase
end
always @(*) begin
case(ALU_op)
2'b00 : vreg = (val_A[15]^~val_B[15])?{1'b0,val_A[14:0]}+{1'b0,val_B[14:0]} : 16'b0000_0000_0000_0000;
2'b01 : begin 
case ({val_A[15],val_B[15]})
2'b01 : vreg = {1'b0,val_A[14:0]}+{1'b0,~val_B[14:0]};
2'b10 : vreg = {1'b0,~val_A[14:0]}+{1'b0,val_B[14:0]};
default : vreg = 16'b0000_0000_0000_0000;
endcase
end
default : vreg = 16'b0000_0000_0000_0000;
endcase
end
endmodule: ALU


module ram(input clk, input ram_w_en, input [7:0] ram_r_addr, input [7:0] ram_w_addr,
           input [15:0] ram_w_data, output reg [15:0] ram_r_data);
    reg [15:0] m[255:0];
    always_ff @(posedge clk) begin
        if (ram_w_en) m[ram_w_addr] <= ram_w_data;
        ram_r_data <= m[ram_r_addr];
    end
    initial $readmemb("ram_init.txt", m);
endmodule: ram
