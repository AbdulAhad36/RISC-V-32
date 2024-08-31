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
