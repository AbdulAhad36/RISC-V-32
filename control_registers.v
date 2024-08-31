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


// EXECUTE REGISTER
module execute_register (
	input clk,regwrite_D,memwrite_D,jump_D,branch_D,alusrc_D,
	input [1:0] resultsrc_D,
	input [2:0] alucontrol_D,
	input [31:0] RD1,RD2,PC_D,immext_D,PCplus_4D,
	input [4:0] rs1_D,rs2_D,rd_D, 
	input flush_E,
	output reg regwrite_E,memwrite_E,jump_E,branch_E,alusrc_E,
	output reg [1:0] resultsrc_E,
	output reg [2:0] alucontrol_E,
	output reg [31:0] RD1_E,RD2_E,PC_E,immext_E,PCplus_4E,
	output reg [4:0] rs1_E,rs2_E,rd_E	
	);

	always@(posedge clk) begin
		if (flush_E) begin
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

//MEMORY REGISTER
module memory_register(
	input clk,regwrite_E,memwrite_E,
	input [1:0] resultsrc_E,
	input [4:0] rd_E,
	input [31:0] aluresult_E,writedata_E,PCplus_4E,
	output reg regwrite_M,memwrite_M,
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
