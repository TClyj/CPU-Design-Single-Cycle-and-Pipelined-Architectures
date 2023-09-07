
module reg_file(input logic clk, wr_en,
					 input logic [31:0] write_data,
				    input logic [3:0] write_addr, input logic [3:0]
					 read_addr1, read_addr2, output logic [31:0]
					 read_data1, read_data2);

		logic [15:0][31:0] memory;
		
	always_ff @(posedge clk) begin
		if (wr_en) begin
			memory[write_addr] <= write_data;
		end
	end
	
	assign read_data1 = memory[read_addr1];
	assign read_data2 = memory[read_addr2];
	
endmodule


module reg_file_testbench();
	
//	logic clk, wr_en, write_data, write_addr, read_addr1, read_addr2, read_data1, read_data2;
//	
//	reg_file dut (clk, wr_en, write_data, write_addr, read_addr1, read_addr2, read_data1, read_data2);
	
	logic  clk, wr_en;
	logic  [31:0] write_data;
	logic  [3:0] write_addr;
	logic  [3:0]  read_addr1, read_addr2;
	logic  [31:0]  read_data1, read_data2;
	
	reg_file dut (.*);
	
	parameter CLOCK_PERIOD=100;
	initial begin
		clk <= 0;
		forever #(CLOCK_PERIOD/2) clk <= ~clk; 
	end

	initial begin
		repeat(1) @(posedge clk);
		wr_en <= 0;repeat(1) @(posedge clk);
		wr_en <= 1;repeat(1) @(posedge clk);
		write_data <= 32'd8; write_addr <= 4'd1;repeat(1) @(posedge clk); 
	   read_addr1 <= 4'd1;repeat(1) @(posedge clk); 
		repeat(10) @(posedge clk);
//		wr_en = 1; #10;
//		write_data = 32'd1; write_addr = 4'd1; cin = 0; #10;
	
	
//		A = 0; B = 0; cin = 0; #10;
//		              cin = 1; #10;
//				 B = 1; cin = 0; #10;
//		              cin = 1; #10;
//		A = 1; B = 0; cin = 0; #10;
//		              cin = 1; #10;
//				 B = 1; cin = 0; #10;
//		              cin = 1; #10;
						  
		$stop;
		
		end 
		
endmodule 