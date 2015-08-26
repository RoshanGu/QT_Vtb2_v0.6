(* blackbox *)
module multiply(a, b, p);
parameter A_WIDTH = 25;
parameter B_WIDTH = 18;
parameter Y_WIDTH = A_WIDTH+B_WIDTH;
input [A_WIDTH-1:0] a;
input [B_WIDTH-1:0] b;
output [Y_WIDTH-1:0] p;

/*
assign out = a * b;
*/

endmodule
