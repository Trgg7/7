module tb_task3(output err);
  // your implementation here
reg clk;
reg rst_n;
reg [7:0] start_pc;
wire [15:0] out;

reg error;
assign err = error;
integer n_pass;
integer n_fail;

task3 dut(.clk, .rst_n, .start_pc, .out);
always begin #5 clk=~clk; end
initial begin
error=0;
n_pass=0;
n_fail=0;

clk=0;
rst_n=1'b0;
start_pc=8'd0;

#10;

rst_n <= 1'b1;
#20;
assert (out === 16'bxxxx_xxxx_xxxx_xxxx) begin
$display("pass halt & fetch");
n_pass = n_pass + 1;
end else begin
$error("fail on halt & fetch");
error = 1'b1;
n_fail = n_fail +1;
end
//get instruc
#5;
start_pc=8'd1;
rst_n=1'b0;
#25;
rst_n=1'b1;
#60;
//state finished

#60;
assert (out === 16'b0000_0000_0000_0010) begin
$display("pass shiftleft");
n_pass = n_pass + 1;
end else begin
$error("fail shiftleft");
error = 1'b1;
n_fail = n_fail +1;
end

#80;
//260
//next state finished
assert (out === 16'b0000_0000_0000_0011) begin
$display("pass add");
n_pass = n_pass + 1;
end else begin
$error("fail add");
error = 1'b1;
n_fail = n_fail +1;
end

#80;
//340
assert (out === 16'b0000_0000_0000_0011) begin
$display("pass and");
n_pass = n_pass + 1;
end else begin
$error("fail and");
error = 1'b1;
n_fail = n_fail +1;
end
//3rd instruction in


#60;
assert (out === 16'b1111_1111_1111_1100) begin
$display("pass inverted shift");
n_pass = n_pass + 1;
end else begin
$error("fail inverted shift");
error = 1'b1;
n_fail = n_fail +1;
end
//3rd state finished

#180;
//580
assert (out === 16'b0000_0000_0000_0010) begin
$display("pass ldr");
n_pass = n_pass + 1;
end else begin
$error("fail str");
error = 1'b1;
n_fail = n_fail +1;
end
#60
assert (out === 16'b0000_0000_0000_0101) begin
$display("pass load str");
n_pass = n_pass + 1;
end else begin
$error("fail load str");
error = 1'b1;
n_fail = n_fail +1;
end
#40
assert (out === 16'b1100_0000_0010_1000) begin
$display("pass store str");
n_pass = n_pass + 1;
end else begin
$error("fail store str");
error = 1'b1;
n_fail = n_fail +1;
end
#100;
$display("\n\n==== TEST SUMMARY ====");
$display("  TEST COUNT: %-5d", n_pass + n_fail);
$display("    - PASSED: %-5d", n_pass);
$display("    - FAILED: %-5d", n_fail);
$display("======================\n\n");

$stop;
end

endmodule: tb_task3
