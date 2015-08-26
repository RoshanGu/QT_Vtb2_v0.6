module adder(a, b, cin, cout, sumout);
input a, b, cin;
output cout, sumout;
xadder _TECHMAP_REPLACE_ (.a_xor_b(a^b), .a_and_b(a&b), .cin(cin), .cout(cout), .sumout(sumout));
endmodule
