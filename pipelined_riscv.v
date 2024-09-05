#developed by Muhammad Abdul Ahad

module pipelined_riscv(clk, rst);
  input clk, rst;

  wire [31:0] PC_F_IN, PC_F_OUT , PCplus_4F, instruction; 	 //FETCH STAGE SIGNALS

  wire [31:0] instr_D,PCplus_4D,PC_D,RD1,RD2,immext_D; 		//DECODE STAGE SIGNALS
  wire branch_D,jump_D,memwrite_D,alusrc_D,regwrite_D;
  wire [1:0] resultsrc_D,immsrc_D,alu_op;
  wire [2:0] alucontrol_D;

  wire branch_E,jump_E,memwrite_E,alusrc_E,regwrite_E,zero_E;	// EXECUTE STAGE SIGNALS
  wire [1:0] resultsrc_E;
  wire [2:0] alucontrol_E;
  wire [4:0] rs1_E, rs2_E,rd_E;
  wire [31:0] immext_E,PCplus_4E,PC_E,RD1_E,RD2_E,PCtarget_E,aluresult_E;  
  wire [31:0] srcA_E,srcB_E,outputforwadBE;
  wire PCsrc_E;

  wire regwrite_M,memwrite_M;					// MEMORY STAGE SIGNALS
  wire [1:0] resultsrc_M;
  wire [4:0] rd_M;
  wire [31:0] aluresult_M,writedata_M,PCplus_4M,readdata_M;

  wire regwrite_W;
  wire [1:0] resultsrc_W;					//WRITE BACK STAGE SIGNALS
  wire [31:0] result_W,aluresult_W,readdata_W,PCplus_4W;
  wire [4:0]rd_W;

  wire stall_F;
  wire stall_D;						// HAZARD CONTROL SIGNALS
  wire flush_D;
  wire flush_E;
  wire [1:0] forward_AE; 
  wire [1:0] forward_BE;

  
  
  //MUX (PCF')
  mux_pcf pcf_out(
	.PCplus_4F(PCplus_4F),
	.PCtarget_E(PCtarget_E), 
	.PCsrc_E(PCsrc_E), 
	.PC_F_IN(PC_F_IN)
	);
 //PRAGRAM COUNTER WITH STALL F
  Program_Counter P_C(
	.clk(clk), 
	.rst(rst), .stall_F(stall_F),
	.PC_F_IN(PC_F_IN), 
	.PC_F_OUT(PC_F_OUT)
	); 
  //ADDER FOR PCPLUS4F
  pcplus pclus (
  	 .PC_F_OUT(PC_F_OUT), 
	 .b(32'd4),
	 .PCplus_4F(PCplus_4F)
	);
  //INSTRUCTION MEMORY
  instruction_memory im (
	.inst_addr(PC_F_OUT),	
	.instruction(instruction)
	);
  //DECODE REGISTER
  decode_register decodereg(
	 .clk(clk),
	 .RD(instruction),
	 .stall_D(stall_D),
	 .flush_D(flush_D),
	 .PC_F(PC_F_OUT),
	 .PCplus_4F(PCplus_4F),
	 .instr_D(instr_D),
	 .PCplus_4D(PCplus_4D),
	 .PC_D(PC_D)
 );

  //REGISTER FILE
  registerFile rf (
	 .WD3(result_W),
 	 .rs1(instr_D[19:15]), 
	 .rs2(instr_D[24:20]), 
	 .rd(rd_W),
 	 .write_enable(regwrite_W), 
	 .clk(clk),
 	 .RD1(RD1), 
	 .RD2(RD2)
	);

  //IMMEDIATE EXTENDER
  imm_ext imext (
	.instruction(instr_D[31:7]),
  	.immSrc(immsrc_D),
  	.immext_D(immext_D)
	);
 
  //MAIN DECODER(CONTROL UNIT)
  main_decoder maindec(
	.opcode(instr_D[6:0]),
	.branch_D(branch_D),
	.jump_D(jump_D),
	.resultsrc_D(resultsrc_D),
	.memwrite_D(memwrite_D),
	.alusrc_D(alusrc_D),
	.immsrc_D(immsrc_D),
	.regwrite_D(regwrite_D),
	.alu_op
	);

// ALU Decoder
  alu_decoder alu_dec(
    	.func3(instr_D[14:12]),
   	.func7(instr_D[30]),
    	.aluop_D(alu_op),
    	.alucontrol_D(alucontrol_D)
	);
 // PCSOURCE_E
  pcsource pcsrc_e(
	.branch_E(branch_E),
	.zero_E(zero_E),
	.jump_E(jump_E),
	.PCsrc_E(PCsrc_E)
	);
// EXECUTE REGISTER
 execute_register executereg(
	.clk(clk),
	.regwrite_D(regwrite_D),.memwrite_D(memwrite_D),.jump_D(jump_D),.branch_D(branch_D),.alusrc_D(alusrc_D),
	.resultsrc_D(resultsrc_D),
	.alucontrol_D(alucontrol_D),
	.RD1(RD1),.RD2(RD2),	
	.PC_D(PC_D),
	.rs1_D(instr_D[19:15]),
	.rs2_D(instr_D[24:20]),
	.rd_D(instr_D[11:7]),
	.immext_D(immext_D),
	.PCplus_4D(PCplus_4D),
	.flush_E(flush_E),
	.regwrite_E(regwrite_E),.memwrite_E(memwrite_E),.jump_E(jump_E),.branch_E(branch_E),.alusrc_E(alusrc_E),
	.resultsrc_E(resultsrc_E),
	.alucontrol_E(alucontrol_E),
	.RD1_E(RD1_E),.RD2_E(RD2_E),
	.PC_E(PC_E),
	.rs1_E(rs1_E),
	.rs2_E(rs2_E),
	.rd_E(rd_E),
	.immext_E(immext_E),
	.PCplus_4E(PCplus_4E)	
	);
  // MULTIPLEXER FORWARD AE
 forwardAE frwardae(
  	.RD1_E(RD1_E), 
	.result_W(result_W), 
	.aluresult_M(aluresult_M),
 	.forward_AE(forward_AE),
  	.srcA_E(srcA_E)
	);
  // MULTIPLEXER FORWARD BE
 forwardBE frwardBe(
  	.RD2_E(RD2_E), 
	.result_W(result_W), 
	.aluresult_M(aluresult_M),
 	.forward_BE(forward_BE),
  	.outputforwadBE(outputforwadBE)
	);
 // MUX (ALUSRC E)
 alusrcE aluSRCE(
 	.I1(outputforwadBE),
	.I2(immext_E),
  	.s(alusrc_E),
  	.srcB_E(srcB_E)
);
  //ADDER FOR PCTARGETE
pctarget pctarg (
	.PC_E(PC_E),
	.immext_E(immext_E),
	.PCtarget_E(PCtarget_E)
	);
 //ALU
ALU aluu(
	.a(srcA_E),
	.b(srcB_E),
	.op(alucontrol_E),
	.aluresult_E(aluresult_E),
	.zero_E(zero_E)
	);
// MEMORY REGISTER
memory_register memreg(
	.clk(clk),.regwrite_E(regwrite_E),.memwrite_E(memwrite_E),
	.resultsrc_E(resultsrc_E),.rd_E(rd_E),
	.aluresult_E(aluresult_E),.writedata_E(outputforwadBE),.PCplus_4E(PCplus_4E),
	.regwrite_M(regwrite_M),.memwrite_M(memwrite_M),
	.resultsrc_M(resultsrc_M),.rd_M(rd_M),
	.aluresult_M(aluresult_M),.writedata_M(writedata_M),.PCplus_4M(PCplus_4M)
	);

//DATA MEMORY
data_memory dm(
   	.clk(clk),.memwrite_M(memwrite_M),
   	.address(aluresult_M),.WD(writedata_M),
   	.readdata_M(readdata_M)
	);

//WRITE BACK REGISTER
writeback_register writebackreg(
	.clk(clk),.regwrite_M(regwrite_M),
	.resultsrc_M(resultsrc_M),
	.rd_M(rd_M),
	.aluresult_M(aluresult_M),.readdata_M(readdata_M),.PCplus_4M(PCplus_4M),
	.regwrite_W(regwrite_W),
	.resultsrc_W(resultsrc_W),
	.rd_W(rd_W),
	.aluresult_W(aluresult_W),.readdata_W(readdata_W),.PCplus_4W(PCplus_4W)
	);

//MUX (RESLUT SRC W)
resultsrcw resut_src_w(
  	.I1(aluresult_W),.I2(readdata_W),.I3(PCplus_4W),
 	.s(resultsrc_W),
  	.Out(result_W)
	);
// HAZARD UNIT
hazard_unit hu(
	.rs1_D(instr_D[19:15]),.rs2_D(instr_D[24:20]),.rd_E(rd_E),.rs1_E(rs1_E),.rs2_E(rs2_E),.rd_M(rd_M),.rd_W(rd_W),
	.PCsrc_E(PCsrc_E),.regwrite_M(regwrite_M),.regwrite_W(regwrite_W),
	.resultsrc_E(resultsrc_E),  
	.stall_F(stall_F),.stall_D(stall_D),	
	.flush_D(flush_D),.flush_E(flush_E),	
	.forward_AE(forward_AE),.forward_BE(forward_BE)	
 );
endmodule

// MUX FOR PCF
module mux_pcf	(
	input [31:0]PCplus_4F, PCtarget_E,
  	input PCsrc_E,
 	output reg [31:0]PC_F_IN
 );

  always @(*)
  begin
    if (PCsrc_E == 0)
      PC_F_IN = PCplus_4F;
    else
      PC_F_IN = PCtarget_E;
  end
endmodule

// Program Counter
module Program_Counter(
 	input clk, rst,stall_F,
 	input [31:0]PC_F_IN,
  	output reg [31:0]PC_F_OUT
);
  always @(posedge clk)
  begin
    if (rst) begin
      PC_F_OUT <= 0;
	end
    else if( !rst & !stall_F)
      PC_F_OUT <= PC_F_IN;
  end
endmodule


// ADDER FRO PCPLUS4F
module pcplus(
  input [31:0]PC_F_OUT, b,
  output reg [31:0]PCplus_4F
	);
  always @(*)
  begin
    PCplus_4F = PC_F_OUT + b;
  end
endmodule


// Instruction Memory
module instruction_memory(
  input [31:0] inst_addr,
  output [31:0] instruction
 	);

 reg [7:0] imem [39:0];
   initial  
     begin 
	
 //Main: addi x7, x3,-9			x7=12-9=3
   imem[0]= 8'b10010011; //93
   imem[1]= 8'b10000011; //83
   imem[2]= 8'b01110001; //71
   imem[3]= 8'b11111111; //ff
   //sw x4, x3(0)			Mem[x3+0]=x4   Mem=12  x4=3
   imem[4]= 8'b00100011; //23
   imem[5]= 8'b10100000; //a0
   imem[6]= 8'b01000001; //41
   imem[7]= 8'b00000000; //00
   //or x4,x7,x2			x4=x7 | x2    x7=3  x2=5  x4= 7
   imem[8]= 8'b00110011; //33
   imem[9]= 8'b11100010; //e2
   imem[10]= 8'b00100011; //23
   imem[11]= 8'b00000000; //00
   //and x5, x3, x4			x5= x3 & x4  x4=7 x3=12  x5= 4
   imem[12]= 8'b10110011; //b3
   imem[13]= 8'b11110010; //f2
   imem[14]= 8'b01000001; //41
   imem[15]= 8'b00000000;  //00
   //beq x5, x7, end			x5!=x7   x5=4 and x7=3
   imem[16]= 8'b01100011; //63
   imem[17]= 8'b10000010; //82
   imem[18]= 8'b01110010; //72
   imem[19]= 8'b00000000; //00
   //lw x3, x3(0)			x3=Mem[x3+0]   mem=12  x3=3
   imem[20]= 8'b10000011; //83
   imem[21]= 8'b10100001; //a1
   imem[22]= 8'b00000001; //01
   imem[23]= 8'b00000000; //00
   //slt x4, x4, x3			x4=0  x4 >x3   x4=7  x3=3
   imem[24]= 8'b00110011; //33
   imem[25]= 8'b00100010; //22
   imem[26]= 8'b00110010; //32
   imem[27]= 8'b00000000; //00
   //beq x4, x0, around			x4=x0  branch to around 
   imem[28]= 8'b01100011; //63
   imem[29]= 8'b00000010; //02
   imem[30]= 8'b00000010; //02
   imem[31]= 8'b00000000; //00
   //around: beq x2, x2, around		x2=x2 branch to around  x2=5
   imem[32]= 8'b01100011; //63
   imem[33]= 8'b00000000; //00
   imem[34]= 8'b00100001; //21
   imem[35]= 8'b00000000; //00
   //end: add x3,x4,x2			isn't executed
   imem[36]= 8'b10110011; //b3
   imem[37]= 8'b00000001; //01
   imem[38]= 8'b00100010; //22
   imem[39]= 8'b00000000; //00  
  end
assign instruction= {imem[inst_addr+3],imem[inst_addr+2],
              imem[inst_addr+1],imem[inst_addr]};
endmodule

// DECODE REGISTER
module decode_register (
	input clk,
	input [31:0]RD,
	input stall_D,
	input flush_D,
	input [31:0] PC_F,
	input [31:0] PCplus_4F,
	output reg [31:0] instr_D,
	output reg [31:0] PCplus_4D,
	output reg [31:0] PC_D

	);
	

	always@(posedge clk) begin
		if(!stall_D && flush_D==0) begin
			instr_D <= RD;
			PC_D <= PC_F;
			PCplus_4D <= PCplus_4F;
		end
		else if (flush_D) begin			
			instr_D <= 0 ;
			PC_D <= 0 ;
			PCplus_4D <= 0 ;
		end
	end

endmodule

// Register File
module registerFile(
  input [31:0] WD3,
  input [4:0] rs1, rs2, rd,
  input write_enable, clk,
  output [31:0] RD1, RD2
);

  reg [31:0] Registers [31:0];
  integer i;
  assign RD1 = Registers[rs1];
  assign RD2 = Registers[rs2];
  
  always @(negedge clk)
  begin
    if (write_enable)
      Registers [rd] = WD3;
  end
  
  initial
  begin
 //   for(i=0; i<32; i = i+1)
  //    Registers[i] = i;
	Registers[0]=32'd0;
	Registers[2]=32'd5;
	Registers[3]=32'd12;
	Registers[4]=32'd3;
	Registers[5]=32'd5;
	Registers[7]=32'd7;
  end
endmodule

 
//Immediate Extender
module imm_ext(
  input [31:7]instruction,
  input [1:0]immSrc,
  output reg [31:0]immext_D
 );

  always @(*)
  begin
    case(immSrc)
      2'b00: immext_D = {{20{instruction[31]}}, instruction[31:20]};
      2'b01: immext_D = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
      2'b10: immext_D = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8],1'b0};
      default: immext_D = 32'b0;
    endcase
  end  
endmodule

// Main Decoder
module main_decoder (
	input [6:0]opcode,
	output reg branch_D,
	output reg jump_D,
	output reg [1:0]resultsrc_D,
	output reg memwrite_D,
	output reg alusrc_D,
	output reg [1:0] immsrc_D,
	output reg regwrite_D,
	output reg [1:0] alu_op
);
	//reg clk;
	always@(*) begin
		case(opcode)
		7'b0000011: begin //lw
			regwrite_D <= 1;
			immsrc_D <= 0;
			alusrc_D <= 1;
			memwrite_D <= 0;
			resultsrc_D <= 2'b01;
			branch_D <= 0;
			alu_op <= 2'b00; 
			jump_D <= 0;
		end

		7'b0100011: begin  //sw
			regwrite_D <= 0;
			immsrc_D <= 2'b01;
			alusrc_D <= 1;
			memwrite_D <= 1;
			resultsrc_D <= 2'bxx;
			branch_D <= 0;
			alu_op <= 2'b0;
			jump_D <= 0;
		end

		7'b0110011: begin	//r_type
			regwrite_D <= 1;
			immsrc_D <= 2'bxx;
			alusrc_D <= 0;
			memwrite_D <= 0;
			resultsrc_D <= 2'b00;
			branch_D <= 0;
			alu_op <= 2'b10;
			jump_D <= 0;
		end

		7'b1100011: begin	//beq
			regwrite_D <= 0;
			immsrc_D <= 2'b10;
			alusrc_D <= 0;
			memwrite_D <= 0;
			resultsrc_D <= 2'bxx;
			branch_D <= 1;
			alu_op <= 2'b01;
			jump_D <= 0;
		end
		
		7'b1101111: begin	//jal
			regwrite_D <= 1;
			immsrc_D <= 2'b11;
			alusrc_D <= 1'bx;
			memwrite_D <= 0;
			resultsrc_D <= 2'b10;
			branch_D <= 0;
			alu_op <= 2'b11;
			jump_D <= 1'b1;
		end
		7'b0010011: begin	//i type
			regwrite_D <= 1;
			immsrc_D <= 2'b00;
			alusrc_D <= 1'b1;
			memwrite_D <= 0;
			resultsrc_D <= 2'b00;
			branch_D <= 0;
			alu_op <= 2'b00;
			jump_D <= 1'b0;
		end
	
		default: begin
			regwrite_D <= 1'b0;
			immsrc_D <= 2'b00;
			alusrc_D <= 1'bx;
			memwrite_D <= 1'b0;
			resultsrc_D <= 2'b00;
			branch_D <= 1'b0;
			alu_op <= 2'b00;
			jump_D <= 1'b0;
		end
			
		endcase	
	end
endmodule  


// ALU Decoder
module alu_decoder(
    input [2:0] func3,
    input func7,
    input [1:0] aluop_D,
    output reg [2:0] alucontrol_D
);
    reg [5:0] conc;

    always@(*) begin
        conc = {aluop_D, func3, func7};
    end

    always @(*) begin
        case(conc)
            6'b000100: begin
                alucontrol_D <= 3'b000;  //add
            end
            6'b000101: begin
                alucontrol_D <= 3'b000;  //add
            end
            6'b010100: begin
                alucontrol_D <= 3'b001; //sub
            end
	    6'b010000: begin
		alucontrol_D <= 3'b001; //beq
	    end
            6'b010101: begin
                alucontrol_D <= 3'b001; //sub
            end
            6'b100000: begin
                alucontrol_D <= 3'b000;  //add
            end
            6'b100001: begin 
                alucontrol_D <= 3'b001; //sub
            end
            6'b100100: begin
                alucontrol_D <= 3'b101; //set less than
            end
            6'b100101: begin
                alucontrol_D <= 3'b101; //set less than
            end
            6'b101100: begin
                alucontrol_D <= 3'b011; //or
            end
            6'b101101: begin
                alucontrol_D <= 3'b011; //or
            end
            6'b101110: begin 
                alucontrol_D <= 3'b010; //and
            end
            6'b101111: begin 
                alucontrol_D <= 3'b010; //and
            end
	    default: begin alucontrol_D <= 3'b000; end
        endcase
    end
endmodule

// PC Source
module pcsource (
	input branch_E,
	input zero_E,
	input jump_E,
	output reg PCsrc_E=0
	);
	reg and_out;
	always@(*) begin
		and_out = branch_E & zero_E;
		PCsrc_E= and_out | jump_E ; end
endmodule

// EXECUTE REGISTER
module execute_register (
	input clk,regwrite_D,memwrite_D,jump_D,branch_D,alusrc_D,
	input [1:0] resultsrc_D,
	input [2:0] alucontrol_D,					//decode stage signals
	input [31:0] RD1,RD2,PC_D,immext_D,PCplus_4D,
	input [4:0] rs1_D,rs2_D,rd_D, 
	input flush_E,
	output reg regwrite_E,memwrite_E,jump_E,branch_E,alusrc_E,
	output reg [1:0] resultsrc_E,					//execute stage signals
	output reg [2:0] alucontrol_E,
	output reg [31:0] RD1_E,RD2_E,PC_E,immext_E,PCplus_4E,
	output reg [4:0] rs1_E,rs2_E,rd_E	
	);

	always@(posedge clk) begin
		if (flush_E) begin					//if flush e=1 all execute values = 0
		 	regwrite_E 	<= 0; memwrite_E 	<= 0;		
			jump_E 		<= 0; branch_E 		<= 0;			
			alusrc_E 	<= 0; resultsrc_E 	<= 0;	  	
		  	alucontrol_E 	<= 0; RD1_E		<= 0;	 	
			RD2_E		<= 0; PC_E		<= 0;			
		 	rs1_E		<= 0; rs2_E		<= 0;		 	
		 	rd_E		<= 0; immext_E		<= 0;		 	
		 	PCplus_4E	<= 0;
		end
		else begin
			regwrite_E 	<= regwrite_D ; 	memwrite_E 	<= memwrite_D;		
			jump_E 		<= jump_D ;		branch_E 	<= branch_D;			
			alusrc_E 	<= alusrc_D ; 		resultsrc_E 	<= resultsrc_D;	  	
		  	alucontrol_E 	<= alucontrol_D ;	RD1_E		<= RD1;	 	
			RD2_E		<= RD2 ; 		PC_E		<= PC_D;			
		 	rs1_E		<= rs1_D ; 		rs2_E		<= rs2_D;		 	
		 	rd_E		<= rd_D ; 		immext_E	<= immext_D;		 	
		 	PCplus_4E	<= PCplus_4D;

		end

	end


endmodule

// Multiplexer  forwardAE
module forwardAE(
  input [31:0]RD1_E, result_W, aluresult_M,	
  input [1:0]forward_AE,
  output reg [31:0]srcA_E
);
  always @(*)
  begin
    if (forward_AE == 00)
      srcA_E = RD1_E;
    else if (forward_AE == 01)
      srcA_E = result_W;
    else if (forward_AE == 2'b10)
      srcA_E = aluresult_M;
    else
      srcA_E = 0;
  end
endmodule

// Multiplexer  forwardBE
module forwardBE(
  input [31:0]RD2_E, result_W, aluresult_M,
  input [1:0]forward_BE,
  output reg [31:0]outputforwadBE
);
  always @(*)
  begin
    if (forward_BE == 00)
      outputforwadBE = RD2_E;
    else if (forward_BE == 01)
      outputforwadBE = result_W;
    else if (forward_BE == 2'b10)
      outputforwadBE = aluresult_M;
    else
      outputforwadBE = 0;
  end
endmodule

// MUX (ALUSRC E)
module alusrcE(
  input [31:0]I1, I2,
  input s,
  output reg [31:0]srcB_E
);
  always @(*)
  begin
    if (s == 0)
      srcB_E = I1;
    else
      srcB_E = I2;
  end
endmodule

//ADDER FOR PCTARGET
module pctarget(
  input [31:0]PC_E, immext_E,
  output reg [31:0]PCtarget_E
	);
  always @(*)
  begin
    PCtarget_E = PC_E + immext_E;
  end
endmodule

// ALU
module ALU(
  input [31:0]a, b,
  input [2:0]op,
  output reg [31:0]aluresult_E,
  output reg zero_E
	);
  always @(*)
  begin
    if (op == 3'b000)
      aluresult_E = a + b;
    else if (op == 3'b001)
      aluresult_E = a - b;
    else if (op == 3'b101)
      aluresult_E = a < b;
    else if (op == 3'b011)
      aluresult_E = a | b;
    else if (op == 3'b010)
      aluresult_E = a & b;
  end
  always @(*)
  begin
    if (aluresult_E == 32'd0)
      zero_E = 1;
    else
      zero_E = 0;
  end
endmodule

//MEMORY REGISTER
module memory_register(
	input clk,regwrite_E,memwrite_E,		//execute stage signals
	input [1:0] resultsrc_E,
	input [4:0] rd_E,
	input [31:0] aluresult_E,writedata_E,PCplus_4E,
	output reg regwrite_M,memwrite_M,			//memory stage signals
	output reg [1:0] resultsrc_M,
	output reg [4:0] rd_M,
	output reg [31:0] aluresult_M,writedata_M,PCplus_4M
	);
	always@(posedge clk) begin
		regwrite_M <= regwrite_E;	memwrite_M <= memwrite_E;
		resultsrc_M <= resultsrc_E;	aluresult_M <= aluresult_E;
		writedata_M <= writedata_E;	rd_M <= rd_E;	PCplus_4M <= PCplus_4E;
	end
endmodule

//DATA MEMORY
module data_memory(
   	input clk,memwrite_M,
   	input [31:0]address,WD,
   	output  [31:0] readdata_M
	);
   reg [7:0] Memory [31:0]; 

  assign readdata_M = {Memory[address + 3], Memory[address + 2], Memory[address + 1], Memory[address]}; 

   always @ (posedge clk)
   begin
     if(memwrite_M)
     	{Memory[address + 3], Memory[address + 2], Memory[address + 1], Memory[address]} <= WD ;
     end
endmodule

//WRITE BACK REGISTER
module writeback_register(
	input clk,regwrite_M,
	input [1:0] resultsrc_M,
	input [4:0] rd_M,
	input [31:0] aluresult_M,readdata_M,PCplus_4M,
	output reg regwrite_W,
	output reg [1:0] resultsrc_W,
	output reg [4:0] rd_W,
	output reg [31:0] aluresult_W,readdata_W,PCplus_4W
	);
	always@(posedge clk) begin
		regwrite_W <= regwrite_M;	
		resultsrc_W <= resultsrc_M;	aluresult_W <= aluresult_M;
		readdata_W <= readdata_M;	rd_W <= rd_M;	PCplus_4W <= PCplus_4M;
	end
endmodule

//MUX (RESLUT SRC W)
module resultsrcw(
  input [31:0]I1, I2, I3,
  input [1:0]s,
  output reg [31:0] Out
	);
  always @(*)
  begin
    if (s == 00)
      Out = I1;
    else if (s == 01)
      Out = I2;
    else if (s == 10)
      Out = I3;
  end
endmodule

module hazard_unit(
	input [4:0] rs1_D,rs2_D,rd_E,rs1_E,rs2_E,rd_M,rd_W,
	input PCsrc_E,regwrite_M, regwrite_W,
	input [1:0] resultsrc_E,  
	output reg stall_F=0,
	output reg stall_D=0,
	output reg flush_D,
	output reg flush_E,
	output reg [1:0]forward_AE,
	output reg [1:0]forward_BE	
 );

	reg lw_stall=0;

	//DATA HAZARD (FROWARDING)
	always@(*) begin	
		//FORWARD AE
		if(((rs1_E == rd_M) & regwrite_M) & (rs1_E !=0)) begin	//forward from memory stage
			forward_AE=10;   
		end
		
		else if(((rs1_E == rd_W) & regwrite_W) & (rs1_E !=0)) begin   // forward from writeback stage
			forward_AE=01;
		end
		
		else begin
			forward_AE=00;					//no forwarding
		end
		//FORWARD BE
		if(((rs2_E == rd_M) & regwrite_M) & (rs2_E !=0)) begin	//forward from memory stage
			forward_BE=10;   
		end
		
		else if(((rs2_E == rd_W) & regwrite_W) & (rs2_E !=0)) begin   // forward from writeback stage
			forward_BE=01;
		end
		
		else begin
			forward_BE=00;					//no forwarding
		end
	end
	
	//  LW hazard 
	always@(*) begin
		if(resultsrc_E[0] & ((rs1_D == rd_E) | (rs2_D == rd_E))) begin //calculating stall
			lw_stall<=1;			 				
		end  	
		else begin 
		lw_stall<=0; end
		flush_E<= lw_stall;						//assigning stall values to hazard signals	
		stall_D<= lw_stall;		
		stall_F<= lw_stall;
		
	end
	
	// BRANCH CONTROL HAZARD
	always@(*) begin
		flush_D <= PCsrc_E;
		flush_E <= lw_stall | PCsrc_E;

	end

endmodule

// TEST BENCH (TOP MODULE)
module tb_pipelined_riscv();
  reg clk, rst;
	
  pipelined_riscv uut(clk, rst);
  
  initial begin
  clk = 1;
  end
  always begin
  #10 clk = ~clk;
  end
  initial
  begin
    rst = 1;
    #10 rst = 0;
   end
endmodule
