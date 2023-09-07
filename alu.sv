// Yijia Lu, Jun Fang
// 04/07/2023
// EE 469
// Lab #1, Task 3

// This alu module take two 32 bits a, b and a ALU control sign as input and output a control
// based result and 4 bit ALU flags. 

// When alu control is 00, result should equal to the addition of a and b.
// When alu control is 01, result should equal to the substraction of a and b.
// When alu control is 10, result should equal to the AND logic result of a and b.
// When alu control is 11, result should equal to the OR logic result of a and b.

// When the output result is negative (last digit is 1), the fourth bit of alu flags
//should equal to 1.

// When the output result is zero (all digits are 0), the third bit of alu flags
//should equal to 1.

// When the output result has a carry out (when add or sub, if there is a 33th bits, 
//there is a carry out), the second bit of alu flags should equal to 1.

// When the output result has a overflow (when add or sub, if the result can't be represent 
//in given range, there is a overflow), the first bit of alu flags should equal to 1.



module alu(
  input logic [31:0] a,
  input logic [31:0] b,
  input logic [1:0] ALUControl,
  output logic [31:0] Result,
  output logic [3:0] ALUFlags
);

  logic [31:0] sum;
  logic [31:0] and_result;
  logic [31:0] or_result;
  
  logic [31:0] c; // intermediate carry value
  logic [31:0] c_out; // carry out bit

  always_comb begin
    
	 //Code for situation of substraction
	 if (ALUControl[0] == 1) begin
        sum = a + ~b + 1; //Substraction
		  c[0] = 1;
		  for (int i = 0; i < 31; i++) begin
          c[i+1] = (a[i] & ~b[i]) | (a[i] & c[i]) | (~b[i] & c[i]);
        end
	 
	 //Code for situation of addition
    end else begin
        sum = a + b; //Addition
		  c[0] = 0;
		  for (int i = 0; i < 31; i++) begin
          c[i+1] = (a[i] & b[i]) | (a[i] & c[i]) | (b[i] & c[i]); //the last digit
        end
    end
    
	 //bit for determine if carry out
    c_out = c[31];
	 
    // Bitwise AND
    and_result = a & b;
    // Bitwise OR
    or_result = a | b;
    
	 //Different output result based on different ALU control input
    case (ALUControl)
      2'b00: Result = sum; // Addition
      2'b01: Result = sum; // Subtraction
      2'b10: Result = and_result; // Bitwise AND
      2'b11: Result = or_result;  // Bitwise OR
      default: Result = 0;        // Invalid ALUControl
    endcase

    // Set ALUFlags based on the operation performed	 
    ALUFlags[0] = (sum[31] ^ a[31]) & (~(a[31] ^ b[31] ^ ALUControl[0])) 
	                & (ALUControl[1] == 0); // Overflow flag
						 
    ALUFlags[1] = (ALUControl[1] == 0) & (c_out == 1); // Carry out flag
	 
	 ALUFlags[2] = (Result == 0);          // Zero flag
	 
    ALUFlags[3] = (Result[31] == 1);      // Negative flag
	 
  end

endmodule

// Testbench for ALU function, our test data will be based on the text (TV) table
// First we test all the inputs from table without using textvectors to check if our function
// work correctly. Then we use readmemh to read our alu.tv file to test for our function again.
// All code for testvectors are from the tutorial video.

module alu_testbench();

   logic clk;
   logic [31:0] a, b, Result;
	logic [1:0] ALUControl;
	logic [3:0] ALUFlags;
	logic [103:0] testvectors [1000:0];
	
	alu dut (.a(a), .b(b), .ALUControl(ALUControl), .Result(Result), .ALUFlags(ALUFlags));
	
	parameter CLOCK_PERIOD=100;
	
	initial clk = 1;
	always begin
	  #(CLOCK_PERIOD/2);
	  clk = ~clk; 
	end
	
	initial begin
	  //add
	  ALUControl <= 2'b00; a <= 32'hAB23; b <= 32'hFE8C; repeat(10) @(posedge clk);
	  
	  ALUControl <= 2'b11; a <= 32'hF3C2; b <= 32'h41EA; repeat(10) @(posedge clk);
	  
	  ALUControl <= 2'b10; a <= 32'hB36A; b <= 32'h4C95; repeat(10) @(posedge clk);
	  
	  ALUControl <= 2'b01; a <= 32'h1D2A; b <= 32'h3BC4; repeat(10) @(posedge clk);
//	  
//	  //sub
//	  ALUControl <= 2'b01; a <= 32'h00000000; b <= 32'h00000000; repeat(10) @(posedge clk);
//	  
//	  ALUControl <= 2'b01; a <= 32'h00000000; b <= 32'hFFFFFFFF; repeat(10) @(posedge clk);
//	  
//	  ALUControl <= 2'b01; a <= 32'h00000001; b <= 32'h00000001; repeat(10) @(posedge clk);
//	  
//	  ALUControl <= 2'b01; a <= 32'h00000100; b <= 32'h00000001; repeat(10) @(posedge clk);
//	  
//	  //and
//	  ALUControl <= 2'b10; a <= 32'hFFFFFFFF; b <= 32'hFFFFFFFF; repeat(10) @(posedge clk);
//	  
//	  ALUControl <= 2'b10; a <= 32'hFFFFFFFF; b <= 32'h12345678; repeat(10) @(posedge clk);
//	  
//	  ALUControl <= 2'b10; a <= 32'h12345678; b <= 32'h87654321; repeat(10) @(posedge clk);
//	  
//	  ALUControl <= 2'b10; a <= 32'h00000000; b <= 32'hFFFFFFFF; repeat(10) @(posedge clk);
//	  
//	  //or
//	  ALUControl <= 2'b11; a <= 32'hFFFFFFFF; b <= 32'hFFFFFFFF; repeat(10) @(posedge clk);
//	  
//	  ALUControl <= 2'b11; a <= 32'h12345678; b <= 32'h87654321; repeat(10) @(posedge clk);
//	  
//	  ALUControl <= 2'b11; a <= 32'h00000000; b <= 32'hFFFFFFFF; repeat(10) @(posedge clk);
//	  
//	  ALUControl <= 2'b11; a <= 32'h00000000; b <= 32'h00000000; repeat(10) @(posedge clk);
	
	  $stop();
   end
	
//	initial begin
//	  $readmemh("alu.tv", testvectors);
//	  
//	  for (int i = 0; i < 20; i = i + 1) begin
//	    {ALUControl, a, b, Result, ALUFlags} = testvectors[i]; @(posedge clk);
//	  end
//	end
	
endmodule
