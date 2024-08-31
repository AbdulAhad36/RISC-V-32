module tb_control_unit();
	reg zero;
	reg [6:0] opcode;
	reg [2:0] func3;
	reg [6:0] func7;
	wire pc_src;
	wire  result_src;
	wire  mem_write;
	wire  alu_src;
	wire  [1:0] imm_src;
	wire  reg_write;
	wire  [2:0]alu_control;
	wire branch;
	wire [1:0]alu_op;

	control_unit cu(
		zero,
		opcode,
		func3,
		func7,
		pc_src,
		result_src,
		mem_write,
		alu_src,
		imm_src,
		reg_write,
		alu_control,
		branch,
		alu_op
	);
	

	initial begin
		    {zero, opcode, func3, func7} = 18'b000000110000000000;	 //lw
 		#10 {zero, opcode, func3, func7} = 18'b001000110000000000;  //sw
   		#10 {zero, opcode, func3, func7} = 18'b001100110000000000;  //R-type (add)
  		#10 {zero, opcode, func3, func7} = 18'b001100110000010000;  //R-type (add)
   		#10 {zero, opcode, func3, func7} = 18'b001100110000010000;  //R-type (sub)
   		#10 {zero, opcode, func3, func7} = 18'b001100110100000000;  //R-type (slt)
   		#10 {zero, opcode, func3, func7} = 18'b001100111100000000;  //R-type (or)
   		#10 {zero, opcode, func3, func7} = 18'b001100111110000000;  //R-type (and)
		#10 {zero, opcode, func3, func7} = 18'b111000110000000000;  //beq
	end
		


endmodule
