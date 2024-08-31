module hazard_unit(
	input [4:0] rs1_D,
	input [4:0] rs2_D,
	input [4:0] rd_E,
	input [4:0] rs1_E,	
	input [4:0] rs2_E,
	input PCsrc_E,
	input [1:0] resultsrc_E,
	input [4:0]  rd_M,
	input regwrite_M,
	input [4:0] rd_W,
	input regwrite_W,
	output reg stall_F=0,
	output reg stall_D=0,
	output reg flush_D,
	output reg flush_E,
	output reg [1:0]forward_AE,
	output reg [1:0]forward_BE	
 );

	reg lw_stall;

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
		
		lw_stall= resultsrc_E[0] & ((rs1_D == rd_E) | (rs2_D == rd_E));  //calculating stall
		flush_E<= lw_stall;							
		stall_D<= lw_stall;		
		stall_F<= lw_stall;	
			
	end
	
	// BRANCH CONTROL HAZARD
	always@(*) begin
		flush_D <= PCsrc_E;
		flush_E <= lw_stall | PCsrc_E;

	end


endmodule

module tb_hazard_unit();
	reg clk;
	reg [4:0] rs1_D;
	reg [4:0] rs2_D;
	reg [4:0] rd_E;
	reg [4:0] rs1_E;	
	reg [4:0] rs2_E;
	reg PCsrc_E;
	reg [1:0] resultsrc_E;
	reg [4:0]  rd_M;
	reg regwrite_M;
	reg [4:0] rd_W;
	reg regwrite_W;
	wire stall_F;
	wire stall_D;
	wire flush_D;
	wire flush_E;
	wire [1:0] forward_AE;
	wire [1:0] forward_BE;	

	hazard_unit hu (
	 	 rs1_D,
	 	 rs2_D,
		 rd_E,
		 rs1_E,	
		 rs2_E,
		 PCsrc_E,
		 resultsrc_E,
		 rd_M,
		 regwrite_M,
		 rd_W,
		 regwrite_W,
		 stall_F,
		 stall_D,
		 flush_D,
		 flush_E,
		 forward_AE,
		 forward_BE	

	);
	
	initial begin
		clk=0;
	end
	
	always begin
		#5 clk=~clk;
	end
	
	always@(posedge clk) begin
		rs1_E= 10010; rd_M=10010; regwrite_M=1; rs1_D=11111; rd_E=11111; resultsrc_E=2'b01; #10
		rs1_E= 10010; rd_M=10010; regwrite_M=0; rs2_D=11110; rs1_D=00000; rd_E=11110; resultsrc_E=2'b01; #10
		rs1_E= 10010; rd_M=10011; regwrite_M=1; rs1_D=11111; rd_E=11111; resultsrc_E=2'b00;#10
		rs1_E= 10010; rd_W=10010; regwrite_W=1; rs2_D=11110; rs1_D=00000; rd_E=11110; resultsrc_E=2'b10; #10
		rs1_E= 10010; rd_W=10010; regwrite_W=0;  PCsrc_E=1 ;#10
	
		rs2_E= 10010; rd_M=10010; regwrite_M=1; #10
		rs2_E= 10010; rd_M=10010; regwrite_M=0; #10
		rs2_E= 10010; rd_M=10011; regwrite_M=1; #10
		rs2_E= 10010; rd_W=10010; regwrite_W=1; #10
		rs2_E= 10010; rd_W=10010; regwrite_W=0; 

	end

endmodule
	

















