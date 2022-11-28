module tb_task1(output err);
  // your implementation here
reg clk;
reg rst_n;
reg [7:0] start_pc;
wire [15:0] out;

reg error;
assgin err = error;
integer n_pass;
integer n_fail;

task1 dut(.clk, .rst_n, .start_pc, .out);

initial begin
error=0;
n_pass=0;
n_fail=0;

clk=0;
rst_n=0;
start_pc=8'd0;
forever #5 clk=~clk
#10;

rst_n=1;
#20;
//get instruc

#60;
//state finished

#20;
//next instruction in

#60;
//next state finished


#20;
//3rd instruction in


#60;
//3rd state finished
end

assert (out === 16'b0000_0000_0000_0001) begin
$display("pass halt & fetch");
n_pass = n_pass + 1;
end else begin
$error("fail on halt & fetch");
error = 1'b1;
n_fail = n_fail +1;
end



endmodule: tb_task1
