module control_unit (
	input zero,
	input [6:0] opcode,
	input [2:0] func3,
	input [6:0] func7,
	output pc_src,
	output  result_src,
	output  mem_write,
	output  alu_src,
	output  [1:0] imm_src,
	output  reg_write,
	output  [2:0]alu_control,
	output wire branch,
	output wire [1:0]alu_op
);
	

	assign pc_src= zero & branch ;		//pc_src

	main_decoder md (		//calling main decoder
		.opcode(opcode),
		.branch(branch),
		.result_src(result_src),
		.mem_write(mem_write),
		.alu_src(alu_src),
		.imm_src(imm_src),
		.reg_write(reg_write),
		.alu_op(alu_op)
	);
	
	alu_decoder alud (		//calling alu_decoder
		.opcode(opcode),
		.func3(func3),
		.func7(func7),
		.alu_op(alu_op),
		.alu_control(alu_control)
	);

endmodule

//MAIN DECODER
module main_decoder (
	input [6:0] opcode,
	output reg branch,
	output reg result_src,
	output reg mem_write,
	output reg alu_src,
	output reg [1:0] imm_src,
	output reg reg_write,
	output reg [1:0] alu_op
);
	//reg clk;
	always@(*) begin
		case(opcode)
		7'b0000011: begin //lw
			reg_write <= 1;
			imm_src <= 0;
			alu_src <= 1;
			mem_write <= 0;
			result_src <= 1;
			branch <= 0;
			alu_op <= 0;
		end

		7'b0100011: begin  //sw
			reg_write <= 0;
			imm_src <= 2'b01;
			alu_src <= 1;
			mem_write <= 1;
			result_src <= 'x;
			branch <= 0;
			alu_op <= 0;
		end

		7'b0110011: begin	//r_type
			reg_write <= 1;
			imm_src <= 'x;
			alu_src <= 0;
			mem_write <= 0;
			result_src <= 0;
			branch <= 0;
			alu_op <= 2'b10;
		end

		7'b1100011: begin	//beq
			reg_write <= 0;
			imm_src <= 2'b10;
			alu_src <= 0;
			mem_write <= 0;
			result_src <= 'x;
			branch <= 1;
			alu_op <= 2'b01;
		end
	
		default: begin
			reg_write <= 'x;
			imm_src <= 'x;
			alu_src <= 'x;
			mem_write <= 'x;
			result_src <= 'x;
			branch <= 'x;
			alu_op <= 'x;
		end
			
		endcase	
	end
endmodule  

//ALU_DECODER
module alu_decoder(
	input [6:0]opcode,
	input [2:0]func3,
	input [6:0]func7,
	input [1:0]alu_op,
	output reg [2:0]alu_control
);
	reg [6:0]conc;
	initial begin
	assign conc={alu_op,func3,opcode[5],func7[5]};
	end
	always@(*) begin
		case(conc)
		7'b00xxxxx: begin
			alu_control<=3'b000;  //add
		end
		7'b01xxxxx: begin
			alu_control<=3'b001; //sub
		end
		7'b1000000: begin
			alu_control<=3'b000;  //add
		end
		7'b1000001: begin
			alu_control<=3'b000; //add
		end
		7'b1000010: begin
			alu_control<=3'b000; //add
		end
		7'b1000011: begin 
			alu_control<=3'b001; //sub
		end
		7'b10010xx: begin
			alu_control<=3'b101; //set less than
		end
		7'b10110xx: begin
			alu_control<=3'b011; //or
		end
		7'b10111xx: begin 
			alu_control<=3'b010; //and
		end
		endcase
	end
endmodule
