// Yijia Lu, Jun Fang
// 05/5/2023
// EE 469
// Lab #3


/* arm is the spotlight of the show and contains the bulk of the datapath and control logic. 
   This module is split into three parts, the datapath,Hazard and control. 
*/


// Inputs
// clk - system clock
// rst - system reset
// Instr - incoming 32 bit instruction from imem, contains opcode, condition, addresses and or immediates

// Outputs
// ReadData - data read out of the dmem
// WriteData - data to be written to the dmem
// MemWrite - write enable to allowed WriteData to overwrite an existing dmem word
// PC - the current program count value, goes to imem to fetch instruciton
// ALUResult - result of the ALU operation, sent as address to the dmem

module arm (
    input  logic        clk, rst,
    input  logic [31:0] Instr,
    input  logic [31:0] ReadData,
    output logic [31:0] WriteData, 
    output logic [31:0] PC, ALUResult,
    output logic        MemWrite
);

    // datapath buses and signals
    logic [31:0] PCPrime, PCPlus4F, PCPlus8D, PCPlus8E; // pc signals
    logic [ 3:0] RA1, RA2;                  // regfile input addresses
    logic [31:0] RD1, RD2;                  // raw regfile outputs
    logic [ 3:0] ALUFlags;                  // alu combinational flag outputs
    logic [31:0] ExtImm, SrcA, SrcB;        // immediate and alu inputs 
    logic [31:0] Result;                    // computed or fetched value to be written into regfile or pc

    // control signals
    logic PCSrc, MemtoReg, ALUSrc, RegWrite;
    logic [1:0] RegSrc, ImmSrc, ALUControl;
	 
	 // Add a new control signal FlagWrite
	 logic FlagsWrite;
	 
	 // Add a new register FlagsReg
    logic [3:0] FlagsReg;
    
	 // Additional datapath variables for pipelined registers
	 logic [31:0] InstrD, InstrE;
	 logic [3:0] RA1E, RD1E;
	 logic [31:0] RA2E, RD2E;
	 logic [31:0] ExtImmE;
	 logic [31:0] ALUResultE;
	 logic [31:0] WriteDataE;
	 logic [3:0] WA3E, WA3M, WA3W;
	 logic [31:0] ALUOutM, ALUOutW;
	 logic [31:0] ReadDataW;
	 logic [31:0] PCPrime_prime;
	 logic [31:0] SrcAE, SrcBE;
	 
	 // Additional control signals for pipelined registers
	 logic BranchE, BranchD, BranchTakenE;
	 logic PCSrcE, RegWriteE, MemtoRegE, MemWriteE, FlagsWriteE, ALUSrcE;
	 logic PCSrcM, RegWriteM, MemtoRegM, MemWriteM;
	 logic PCSrcW, RegWriteW, MemtoRegW;
	 logic PCSrcE_prime, RegWriteE_prime, MemWriteE_prime;
	 logic [1:0] ALUControlE;
	 logic [3:0] CondE, CondEx, FlagsRegE;
	 
	 // Variables for Hazard unit
	 logic Match_1E_M, Match_1E_W, Match_2E_M, Match_2E_W, Match_12D_E;
	 logic FlushD, FlushE;
	 logic LdrStall, StallD, StallF;
	 logic PCWrPendingF;
	 logic [1:0] ForwardAE, ForwardBE;
	  
	 
    /* The datapath consists of a PC as well as a series of muxes to make decisions about which data words to pass forward and operate on. It is 
    ** noticeably missing the register file and alu, which you will fill in using the modules made in lab 1. To correctly match up signals to the 
    ** ports of the register file and alu take some time to study and understand the logic and flow of the datapath.
    */
    //-------------------------------------------------------------------------------
    //                                      DATAPATH
    //-------------------------------------------------------------------------------


    assign PCPrime = PCSrcW ? Result : PCPlus4F;  // mux, use either default or newly computed value
    assign PCPrime_prime = BranchTakenE ? ALUResultE : PCPrime;
	 assign PCPlus4F = PC + 'd4;                  // default value to access next instruction
    assign PCPlus8D = PCPlus4F;             // value read when reading from reg[15]
	 
	 
	 //First pipelined register (Fetch & Decode stage)
	 always_ff @(posedge clk) begin
	     if (rst | FlushD) begin
			  InstrD <= '0;
		  end else if(~StallD) begin
			  InstrD <= Instr;		  
			  
		  end
	 end

	 
    // update the PC and FlagsReg state at each posedge clock cycle
    always_ff @(posedge clk) begin
        if (rst) begin
		     PC <= '0;
			  
        end else if(~StallF) begin
   		  PC <= PCPrime_prime;
			  
		  end
    end

	 
    // determine the register addresses based on control signals
    // RegSrc[0] is set if doing a branch instruction
    // RefSrc[1] is set when doing memory instructions
    assign RA1 = RegSrc[0] ? 4'd15        : InstrD[19:16];
    assign RA2 = RegSrc[1] ? InstrD[15:12] : InstrD[ 3: 0];
	 

    // Based on the given read address to output the corresponding data.
    // when RegWrite is set true, the memory of reg_file is updated with the provided value 
    // update at the negative edge
    reg_file u_reg_file (
        .clk       (clk), 
        .wr_en     (RegWriteW),
        .write_data(Result),
        .write_addr(WA3W),
        .read_addr1(RA1), 
        .read_addr2(RA2),
        .read_data1(RD1), 
        .read_data2(RD2)
    );

	 
    // two muxes, put together into an always_comb for clarity
    // determines which set of instruction bits are used for the immediate
    always_comb begin
        if      (ImmSrc == 'b00) ExtImm = {{24{InstrD[7]}},InstrD[7:0]};          // 8 bit immediate - reg operations
        else if (ImmSrc == 'b01) ExtImm = {20'b0, InstrD[11:0]};                 // 12 bit immediate - mem operations
        else                     ExtImm = {{6{InstrD[23]}}, InstrD[23:0], 2'b00}; // 24 bit immediate - branch operation
    end
	 
	 
	 // Second pipelined register (Decode & Execute stage)
	 always_ff @(posedge clk) begin
	     if (rst | FlushE) begin
		     
			  PCSrcE <= '0;
			  RegWriteE <= '0;
			  MemtoRegE <= '0;
			  MemWriteE <= '0;
			  ALUSrcE <= '0;
			  ALUControlE <= '0;
			  FlagsWriteE <= '0;
			  BranchE <= '0;
			  
		     WA3E <= '0;
			  ExtImmE <= '0;
			  RD1E <= '0;
			  RD2E <= '0;
			  RA1E <= '0;
			  RA2E <= '0;
			  PCPlus8E <= '0;
			  
		  end else begin
		  
           PCSrcE <= PCSrc;
			  RegWriteE <= RegWrite;
			  MemtoRegE <= MemtoReg;
			  MemWriteE <= MemWrite;
			  ALUSrcE <= ALUSrc;
			  ALUControlE <= ALUControl;
			  FlagsWriteE <= FlagsWrite;
			  BranchE <= BranchD;
			  FlagsRegE <= FlagsReg;
			  
		     WA3E <= InstrD[15:12];
			  ExtImmE <= ExtImm;
			  RD1E <= RD1;
			  RD2E <= RD2;
			  RA1E <= RA1;
			  RA2E <= RA2;
			  InstrE <= InstrD;
			  PCPlus8E <= PCPlus8D;

		  end
	 
	 end

	 
    // WriteData and SrcA are direct outputs of the register file, wheras SrcB is chosen between reg file output and the immediate
    assign WriteDataE = (RA2E == 'd15) ? PCPlus8E : SrcB;           // substitute the 15th regfile register for PC 
    assign SrcA      = (RA1E == 'd15) ? PCPlus8E : RD1E;           // substitute the 15th regfile register for PC 
    assign SrcBE      = ALUSrcE        ? ExtImmE  : WriteDataE;     // determine alu operand to be either from reg file or from immediate
    
	 
	 //The three input mux controlled by input ForwardAE and ForwardBE
	 //Forwarding signal on SrcAE
	 always_comb begin
	     case(ForwardAE)
		      2'b10: SrcAE = ALUOutM;
				2'b01: SrcAE = Result;
				2'b00: SrcAE = SrcA;
				default: SrcAE = 32'bx;
		  endcase
	 end
	 
	 
	 //Forwarding signal on SrcBE
	 always_comb begin
	     case(ForwardBE)
		      2'b10: SrcB = ALUOutM;
				2'b01: SrcB = Result;
				2'b00: SrcB = RD2E;
				default: SrcB = 32'bx;
		  endcase
	 end
	 
	 
    // This alu module take two 32 bits a, b and a ALU control sign as input and output a control
    // based result and 4 bit ALU flags. 
    alu u_alu (
        .a          (SrcAE), 
        .b          (SrcBE),
        .ALUControl (ALUControlE),
        .Result     (ALUResult),
        .ALUFlags   (ALUFlags)
    );
	 
	 
	 // Logic function inside the control unit in Execute stage
	 assign PCSrcE_prime = PCSrcE & CondEx;
	 assign RegWriteE_prime = RegWriteE & CondEx;
	 assign MemWriteE_prime = MemWriteE & CondEx;
	 assign BranchTakenE = BranchE & CondEx;
	 
	 
	 // Update flag register base on the flagwrite signal
	 always_ff @(posedge clk) begin
	     if (rst) FlagsReg <= 0;
		  else if (FlagsWriteE) FlagsReg <= ALUFlags;
	 end
	 
	 
	 // Third pipelined register (Execute & Memory stage)
	 always_ff @(posedge clk) begin
	     if (rst) begin
		  
		     PCSrcM <= '0;
			  RegWriteM <= '0;
			  MemtoRegM <= '0;
			  MemWriteM <= '0;
			  
			  ALUResultE <= '0;
			  WA3M <= '0;
			  WriteData <= '0;
			  
		  end else begin
		  
		     PCSrcM <= PCSrcE_prime;
			  RegWriteM <= RegWriteE_prime;
			  MemtoRegM <= MemtoRegE;
			  MemWriteM <= MemWriteE_prime;
			  
		     WA3M <= WA3E;
			  WriteData <= WriteDataE;
			  ALUResult <= ALUResultE;
			  ALUOutM <= ALUResultE;
			  
		  end
	 
	 end
	 
	 // Fourth stage (Memory & Writeback stage)
	 always_ff @(posedge clk) begin
	    if (rst) begin
		 
		    PCSrcW <= '0;
			 RegWriteW <= '0;
			 MemtoRegW <= '0;
			 
		    WA3W <= '0;
			 ALUOutW <= '0;
			 ReadDataW <= '0; 
			 
		  end else begin
		  
		    PCSrcW <= PCSrcM;
			 RegWriteW <= RegWriteM;
			 MemtoRegW <= MemtoRegM;
			 
		    WA3W <= WA3M;
			 ALUOutW <= ALUOutM;
			 ReadDataW <= ReadData;
			 
		  end
	 end

    // determine the result to run back to PC or the register file based on whether we used a memory instruction
    assign Result = MemtoRegW ? ReadDataW : ALUOutW;    // determine whether final writeback result is from dmemory or alu
    
	 
	 
	 
	 /* The Hazard unit control the forwarding and stalling to deal with data or control hazard.
	 ** Based on the matching result, the unit determines the forwarding signal to mux to choose the 
    ** sources, flush and stall signal to deal with the LDR command and branchtaken
    */
    //-------------------------------------------------------------------------------
    //                                      Hazard Unit
    //-------------------------------------------------------------------------------
	 
	 // Match signal, check if execute stage registers matches Memory or Writeback stage or not.
	 assign Match_1E_M = (RA1E == WA3M);
	 assign Match_2E_M = (RA2E == WA3M);
    assign Match_1E_W = (RA1E == WA3W);
	 assign Match_2E_W = (RA2E == WA3W);
	 
	 // Forwarding for SrcA
    always_comb begin
	     if (Match_1E_M & RegWriteM) ForwardAE = 2'b10;
		  else if (Match_1E_W & RegWriteW) ForwardAE = 2'b01;
		  else ForwardAE = 2'b00;
    end	
	 
	 // Forwarding for SrcB
    always_comb begin
	     if (Match_2E_M & RegWriteM) ForwardBE = 2'b10;
		  else if (Match_2E_W & RegWriteW) ForwardBE = 2'b01;
		  else ForwardBE = 2'b00;
    end	
	 
	 // Stalling logic
	 assign Match_12D_E = (RA1 == WA3E)|(RA2 == WA3E);
	 assign LdrStall = Match_12D_E & MemtoRegE;
	 
	 // Control Stalling Logic
	 assign PCWrPendingF = PCSrc | PCSrcE | PCSrcM;
	 
	 assign StallD = LdrStall;
	 
	 assign StallF = LdrStall | PCWrPendingF;
	 
	 assign FlushD = PCWrPendingF | PCSrcW | BranchTakenE;
	 
	 assign FlushE = LdrStall | BranchTakenE;
	 
	 
	 
	 
    /* The control conists of a large decoder, which evaluates the top bits of the instruction and produces the control bits 
    ** which become the select bits and write enables of the system. The write enables (RegWrite, MemWrite and PCSrc) are 
    ** especially important because they are representative of your processors current state. 
    */
    //-------------------------------------------------------------------------------
    //                                      CONTROL
    //-------------------------------------------------------------------------------
    
	 
    always_comb begin
        casez (InstrD[27:20])

            // ADD (Imm or Reg)
            8'b00?_0100_0 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we add
                PCSrc    = 0;
                MemtoReg = 0; 
                MemWrite = 0; 
                ALUSrc   = InstrD[25]; // may use immediate
                RegWrite = 1;
                RegSrc   = 'b00;
                ImmSrc   = 'b00; 
                ALUControl = 'b00;
					 FlagsWrite = 0;
					 BranchD = 0;
            end

            // SUB (Imm or Reg)
            8'b00?_0010_0 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we sub
                PCSrc    = 0; 
                MemtoReg = 0; 
                MemWrite = 0; 
                ALUSrc   = InstrD[25]; // may use immediate
                RegWrite = 1;
                RegSrc   = 'b00;
                ImmSrc   = 'b00; 
                ALUControl = 'b01;
					 FlagsWrite = 0;
					 BranchD = 0;
            end
				
				// SUBS/CMP (Imm or Reg)
            8'b00?_0010_1 : begin
                PCSrc     = 0;
                MemtoReg  = 0;
                MemWrite  = 0;
                ALUSrc    = InstrD[25];
                RegWrite  = 1;
                RegSrc    = 'b00;
                ImmSrc    = 'b00;
                ALUControl = 'b01;
					 FlagsWrite = 1;        // Always update flags for SUBS/CMP
					 BranchD = 0;
            end

            // AND
            8'b000_0000_0 : begin
                PCSrc    = 0; 
                MemtoReg = 0; 
                MemWrite = 0; 
                ALUSrc   = 0; 
                RegWrite = 1;
                RegSrc   = 'b00;
                ImmSrc   = 'b00;    // doesn't matter
                ALUControl = 'b10;
				    FlagsWrite = 0;
				    BranchD = 0;	 
            end

            // ORR
            8'b000_1100_0 : begin
                PCSrc    = 0; 
                MemtoReg = 0; 
                MemWrite = 0; 
                ALUSrc   = 0; 
                RegWrite = 1;
                RegSrc   = 'b00;
                ImmSrc   = 'b00;    // doesn't matter
                ALUControl = 'b11;
					 FlagsWrite = 0;
					 BranchD = 0;
            end

            // LDR
            8'b010_1100_1 : begin
                PCSrc    = 0; 
                MemtoReg = 1; 
                MemWrite = 0; 
                ALUSrc   = 1;
                RegWrite = 1;
                RegSrc   = 'b10;    // msb doesn't matter
                ImmSrc   = 'b01; 
                ALUControl = 'b00;  // do an add
					 FlagsWrite = 0;
					 BranchD = 0;
            end

            // STR
            8'b010_1100_0 : begin
                PCSrc    = 0; 
                MemtoReg = 0; // doesn't matter
                MemWrite = 1; 
                ALUSrc   = 1;
                RegWrite = 0;
                RegSrc   = 'b10;    // msb doesn't matter
                ImmSrc   = 'b01; 
                ALUControl = 'b00;  // do an add
					 FlagsWrite = 0;
					 BranchD = 0;
            end

            // B 
            8'b1010_???? : begin
					case (InstrD[31:28])					
						// condition check for B to determine PC Src
						//BEQ
						4'b0000 : begin
							PCSrc    = (FlagsRegE[2]); 
							MemtoReg = 0;
							MemWrite = 0; 
							ALUSrc   = 1;
							RegWrite = 0;
							RegSrc   = 'b01;
							ImmSrc   = 'b10; 
							ALUControl = 'b00;  // do an add
							FlagsWrite = 0;
							BranchD = 1;
						end
						
						//BNE
						4'b0001 : begin
							PCSrc    = (~FlagsRegE[2]); 
							MemtoReg = 0;
							MemWrite = 0; 
							ALUSrc   = 1;
							RegWrite = 0;
							RegSrc   = 'b01;
							ImmSrc   = 'b10; 
							ALUControl = 'b00;  // do an add
							FlagsWrite = 0;
							BranchD = 1;
						end
						
						//BGE
						4'b1010 : begin
							PCSrc    = (~(FlagsRegE[3]^FlagsRegE[0])); 
							MemtoReg = 0;
							MemWrite = 0; 
							ALUSrc   = 1;
							RegWrite = 0;
							RegSrc   = 'b01;
							ImmSrc   = 'b10; 
							ALUControl = 'b00;  // do an add
							FlagsWrite = 0;
							BranchD = 1;
						end
						
						//BGT
						4'b1100 : begin
							PCSrc    = ((~(FlagsRegE[3]^FlagsRegE[0]))&(~FlagsRegE[2])); 
							MemtoReg = 0;
							MemWrite = 0; 
							ALUSrc   = 1;
							RegWrite = 0;
							RegSrc   = 'b01;
							ImmSrc   = 'b10; 
							ALUControl = 'b00;  // do an add
							FlagsWrite = 0;
							BranchD = 1;
						end
						
						//BLE
						4'b1101 : begin
							PCSrc    = ((FlagsRegE[3]^FlagsRegE[0])| FlagsRegE[2]); 
							MemtoReg = 0;
							MemWrite = 0; 
							ALUSrc   = 1;
							RegWrite = 0;
							RegSrc   = 'b01;
							ImmSrc   = 'b10; 
							ALUControl = 'b00;  // do an add
							FlagsWrite = 0;
							BranchD = 1;
						end
						
						//BLT
						4'b1011 : begin
							PCSrc    = (FlagsRegE[3]^FlagsRegE[0]); 
							MemtoReg = 0;
							MemWrite = 0; 
							ALUSrc   = 1;
							RegWrite = 0;
							RegSrc   = 'b01;
							ImmSrc   = 'b10; 
							ALUControl = 'b00;  // do an add
							FlagsWrite = 0;
							BranchD = 1;
						end
						
						//unconditional 1110
						default: begin
							PCSrc    = 1; 
							MemtoReg = 0;
							MemWrite = 0; 
							ALUSrc   = 1;
							RegWrite = 0;
							RegSrc   = 'b01;
							ImmSrc   = 'b10; 
							ALUControl = 'b00;  // do an add
							FlagsWrite = 0;
							BranchD = 1;
						end
					endcase
				end

			default: begin
					PCSrc    = 0; 
               MemtoReg = 0; // doesn't matter
               MemWrite = 0; 
               ALUSrc   = 0;
               RegWrite = 0;
               RegSrc   = 'b00;
               ImmSrc   = 'b00; 
               ALUControl = 'b00;  // do an add
				   FlagsWrite = 0;
					BranchD = 0;
						  
			end
        endcase
    end
	 
	 
	 // Cond unit case situations, since it is in a different stage so I seperate
	 // with the previous always_comb begin function
	 always_comb begin
	     case (InstrE[31:28])					
						// condition check for B to determine CondEx situation
						//EQ
						4'b0000 : begin
							CondEx    = (FlagsRegE[2]); 
							
						end
						
						//NE
						4'b0001 : begin
							CondEx    = (~FlagsRegE[2]); 
							
						end
						
						//GE
						4'b1010 : begin
							CondEx    = (~(FlagsRegE[3]^FlagsRegE[0])); 
							
						end
						
						//GT
						4'b1100 : begin
							CondEx    = ((~(FlagsRegE[3]^FlagsRegE[0]))&(~FlagsRegE[2])); 
							
						end
						
						//LE
						4'b1101 : begin
							CondEx    = ((FlagsRegE[3]^FlagsRegE[0])| FlagsRegE[2]); 
							
						end
						
						//LT
						4'b1011 : begin
							CondEx    = (FlagsRegE[3]^FlagsRegE[0]); 
							
						end
						
						//unconditional 1110
						default: begin
							CondEx    = 1; 
							
						end
					endcase
	 end
	 


endmodule
