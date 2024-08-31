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

