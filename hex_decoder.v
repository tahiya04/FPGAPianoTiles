`timescale 1ns/1ns

module hex_decoder(c, display); 
// c is input
	input [3:0] c;
	output [6:0] display;
// c[0] is a
// c[1] is b
// c[2] is c
// c[3] is d
// map c to sw0-1sw1
// display to hex
	wire [15:0]w;
	
	assign w[0] = ~c[3] & ~c[2] & ~c[1] & ~c[0];
	assign w[1] = ~c[3] & ~c[2] & ~c[1] & c[0];
	assign w[2] = ~c[3] & ~c[2] & c[1] & ~c[0];
	assign w[3] = ~c[3] & ~c[2] & c[1] & c[0];
	assign w[4] = ~c[3] & c[2] & ~c[1] & ~c[0];
	assign w[5] = ~c[3] & c[2] & ~c[1] & c[0];
	assign w[6] = ~c[3] & c[2] & c[1] & ~c[0];
	assign w[7] = ~c[3] & c[2] & c[1] & c[0];
	
	
	assign w[8] = c[3] & ~c[2] & ~c[1] & ~c[0];
	assign w[9] = c[3] & ~c[2] & ~c[1] & c[0];
	assign w[10] = c[3] & ~c[2] & c[1] & ~c[0];
	assign w[11] = c[3] & ~c[2] & c[1] & c[0];
	assign w[12] = c[3] & c[2] & ~c[1] & ~c[0];
	assign w[13] = c[3] & c[2] & ~c[1] & c[0];
	assign w[14] = c[3] & c[2] & c[1] & ~c[0];
	assign w[15] = c[3] & c[2] & c[1] & c[0];
	
	assign display[0] = w[1] | w[4] | w[11] | w[13];
	assign display[1] = w[5] | w[6] | w[11] | w[12] | w[14] | w[15];
	assign display[2] = w[2] | w[12] | w[14] | w[15];
	assign display[3] = w[1] | w[4] | w[7]| w[10] | w[15];
	
	assign display[4] = w[1] | w[3] | w[4] | w[5] | w[7] | w[9];
	assign display[5] = w[1] | w[2] | w[3] | w[7] | w[13];
	assign display[6] = w[0] | w[1] | w[7] | w[12];
	
	
	

	

	
	//assign display[0] = (~c[0]&~c[1]&~c[2]&c[3]) | (~c[0]&c[1]&~c[2]&~c[3]) | (c[0]&c[1]&~c[2]&c[3]) | (c[0]&c[1]&c[2]&~c[3]);
	//assign display[1] = (c[0]&c[1]&~c[2]&~c[3]) | (~c[0]&c[1]&~c[2]&c[3]) | (~c[0]&~c[1]&c[2]&c[3]) | (~c[0]&c[1]&c[2]&~c[3]) | (c[0]&c[2]&c[3]);
	//assign display[2] = (c[0]&c[1]&~c[2]&~c[3]) | (~c[0]&~c[1]&c[2]&~c[3]) | (c[0]&c[1]&c[2]);
	//assign display[3] = (~c[0]&c[1]&~c[2]&~c[3]) | (~c[0]&~c[1]&~c[2]&c[3]) | (c[1]&c[2]&c[3]) | (c[0]&~c[1]&c[2]&~c[3]);
	//assign display[4] = (~c[0]&c[1]&~c[2]&~c[3]) | (c[0]&~c[1]&~c[2]&c[3]) | (~c[0]&c[3]);
	//assign display[5] = (~c[0]&~c[1]&c[3]) | (~c[0]&~c[1]&c[2]) | (~c[0]&c[3]) | (c[0]&c[1]&~c[2]&c[3]);
	//assign display[6] = (~c[0]&~c[1]&~c[2]) | (c[0]&c[1]&~c[2]&~c[3]) | (~c[0]&c[1]&c[2]&c[3]);



endmodule 