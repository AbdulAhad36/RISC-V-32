module RISC_V_Processor(clk, rst);
  input clk, rst;

  wire [31:0]PCNext, PC, instr, PCPlus4, PCTarget, SrcA, RD2, ImmExt, SrcB, ALUResult, ReadData, Result;
  wire PCSrc, ResultSrc, MemWrite, ALUSrc, RegWrite, Zero;
  wire [2:0]ALUControl;
  wire [1:0]ImmSrc;
  
  Mux M1(.I1(PCPlus4), .I2(PCTarget), .s(PCSrc), .Out(PCNext));
  Program_Counter P_C(.clk(clk), .PC_In(PCNext), .rst(rst), .PC_Out(PC));
  Adder Add1(.a(PC), .b(32'd4), .out(PCPlus4));
  instruction_memory IM(.inst_addr(PC), .instr(instr));
//  Instr_Fetch IF(.clk(clk), .rst(rst), .instr());
  registerFile RF(.WriteData(Result), .rs1(instr[19:15]), .rs2(instr[24:20]), .rd(instr[11:7]), .RegWrite(RegWrite), .clk(clk), .ReadData1(SrcA), .ReadData2(RD2));
  Mux M2(.I1(RD2), .I2(ImmExt), .s(ALUSrc), .Out(SrcB));
  ALU ALU1(.Result(ALUResult), .a(SrcA), .b(SrcB), .op(ALUControl), .zero(Zero));
  Data_Memory DM(.clk(clk), .Mem_Addr(ALUResult), .Write_Data(RD2), .Mem_Write(MemWrite), .Read_Data(ReadData));
  Mux M3(.I1(ALUResult), .I2(ReadData), .s(ResultSrc), .Out(Result));
  imm_ext IDE(.instruction(instr), .immSrc(ImmSrc), .imm_data(ImmExt));
  Adder Add2(.a(PC), .b(ImmExt), .out(PCTarget));
  Control_Unit CU(.PCSrc(PCSrc), .Zero(Zero), .ResultSrc(ResultSrc), .MemWrite(MemWrite), .ALUSrc(ALUSrc), .ImmSrc(ImmSrc), .ALUControl(ALUControl), .RegWrite(RegWrite), .Opcode(instr[6:0]), .Funct3(instr[14:12]), .Funct7(instr[30]));
endmodule
  
// Multiplexer  
module Mux(I1, I2, s, Out);
  input [31:0]I1, I2;
  input s;
  output reg [31:0]Out;
  always @(*)
  begin
    if (s == 0)
      Out = I1;
    else
      Out = I2;
  end
endmodule

// Instrution Fetch
/*module Instr_Fetch(clk, rst, instr);
  input clk, rst;
  output [31:0]instr;
  wire [31:0]PC_in;
  wire [31:0]PC_out;
  
  Program_Counter P_C(.clk(clk), .PC_In(PC_in), .rst(rst), .PC_Out(PC_out));
  Adder Add(.a(PC_out), .b(32'd4), .out(PC_in));
  instruction_memory IM(.inst_addr(PC_out), .instr(instr));
endmodule */

// Program Counter
module Program_Counter(clk, PC_In, rst, PC_Out);
  input clk, rst;
  input [31:0]PC_In;
  output reg [31:0]PC_Out;
  always @(posedge clk)
  begin
    if (rst)
      PC_Out <= 0;
    else 
      PC_Out <= PC_In;
  end
endmodule

// Adder
module Adder(a, b, out);
  input [31:0]a, b;
  output reg [31:0]out;
  always @(*)
  begin
    out = a + b;
  end
endmodule

// Instruction Memory
module instruction_memory(inst_addr,instr);
  input [31:0] inst_addr;
  output [31:0] instr;
  reg [7:0] imem [15:0];
  initial  
  begin 
   imem[0]= 8'b00000011; //03
   imem[1]= 8'b10100011; //a3
   imem[2]= 8'b11000100; //c4
   imem[3]= 8'b11111111; //ff
   imem[4]= 8'b00100011; //23
   imem[5]= 8'b10100100; //a4
   imem[6]= 8'b01100100; //64
   imem[7]= 8'b00000000; //00
   imem[8]= 8'b00110011; //33
   imem[9]= 8'b11100010; //e2
   imem[10]= 8'b00110010; //32
   imem[11]= 8'b00000000; //00
   imem[12]= 8'b11100011; //e3
   imem[13]= 8'b00001010; //0a
   imem[14]= 8'b01000010; //42
   imem[15]= 8'b11111110;  //fe
end 
assign instr= {imem[inst_addr+3],imem[inst_addr+2],
              imem[inst_addr+1],imem[inst_addr]};
    
endmodule

// Register File
module registerFile(WriteData, rs1, rs2, rd, RegWrite, clk, ReadData1, ReadData2);
  input [31:0] WriteData;
  input [4:0] rs1, rs2, rd;
  input RegWrite, clk;
  output [31:0] ReadData1, ReadData2;
  reg [31:0] Registers [31:0];
  integer i;
  assign ReadData1 = Registers[rs1];
  assign ReadData2 = Registers[rs2];
  
  always @(posedge clk)
  begin
    if (RegWrite)
      Registers [rd] = WriteData;
  end
  
  initial
  begin
    for(i=0; i<32; i = i+1)
      Registers[i] = i;
  end
endmodule

// ALU
module ALU(Result, a, b, op, zero);
  input [31:0]a, b;
  input [2:0]op;
  output reg [31:0]Result;
  output reg zero;
  always @(*)
  begin
    if (op == 3'b000)
      Result = a + b;
    else if (op == 3'b001)
      Result = a - b;
    else if (op == 3'b101)
      Result = a < b;
    else if (op == 3'b011)
      Result = a | b;
    else if (op == 3'b010)
      Result = a & b;
  end
  always @(*)
  begin
    if (Result == 32'd0)
      zero = 1;
    else
      zero =  0;
  end
endmodule

// Data Memory
module Data_Memory(clk, Mem_Addr, Write_Data, Mem_Write, Read_Data);
   input clk;
   input [31:0]Mem_Addr;
   input [31:0]Write_Data;
   input Mem_Write;
   output [31:0] Read_Data;
   reg [7:0] Memory [31:0]; //[15:0]
   integer i;
   assign Read_Data = {Memory[Mem_Addr + 3], Memory[Mem_Addr + 2], Memory[Mem_Addr + 1], Memory[Mem_Addr + 0]};

//The below block will be changes to the values given in the manual.
   initial
   begin
     for (i=0; i<32; i = i + 1)
       Memory [i] = i;
    end
     
   always @ (posedge clk)
   begin
     if(Mem_Write)
       Memory[Mem_Addr] = Write_Data[7:0];
       Memory[Mem_Addr + 1] = Write_Data[15:8];
       Memory[Mem_Addr + 2] = Write_Data[23:16];
       Memory[Mem_Addr + 3] = Write_Data[31:24];
     end
  endmodule

//Immediate Extender
module imm_ext(instruction, immSrc, imm_data);
  input [31:0]instruction;
  input [1:0]immSrc;
  output reg [31:0]imm_data;
  always @(*)
  begin
    case(immSrc)
      2'b00: imm_data = {{20{instruction[31]}}, instruction[31:20]};
      2'b01: imm_data = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
      2'b10: imm_data = {{21{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8],1'b0};
      default: imm_data = 32'b0;
    endcase
  end  
endmodule

module Control_Unit(PCSrc, Zero, ResultSrc, MemWrite, ALUSrc, ImmSrc, ALUControl, RegWrite, Opcode, Funct3, Funct7);
  input [2:0]Funct3;
  input Funct7, Zero;
  input [6:0]Opcode;
  output ResultSrc, MemWrite, ALUSrc, RegWrite, PCSrc;
  output [1:0]ImmSrc;
  output [2:0]ALUControl;
  wire Branch;
  wire [1:0]ALUOp;

  main_decoder CU1(.branch(Branch), .opcode(Opcode), .imm_src(ImmSrc), .result_src(ResultSrc), .alu_op(ALUOp), .mem_write(MemWrite), .alu_src(ALUSrc), .reg_write(RegWrite));
  alu_decoder CU2(.alu_op(ALUOp), .func7(Funct7), .func3(Funct3), .alu_control(ALUControl));
  PC_Src CU3(.branch(Branch), .zero(Zero), .pc_src(PCSrc));
endmodule

// Main Decoder
module main_decoder (
	input [6:0]opcode,
	input zero,
	output reg branch,
	output reg result_src,
	output reg mem_write,
	output reg alu_src,
	output reg [1:0] imm_src,
	output reg reg_write,
	output reg [1:0] alu_op,
	output pc_src
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

// ALU Decoder
module alu_decoder(
    input [2:0] func3,
    input func7,
    input [1:0] alu_op,
    output reg [2:0] alu_control
);
    reg [6:0] conc;

    initial begin
        assign conc = {alu_op, func3, func7};
    end

    always @(*) begin
        case(conc)
            6'b000100: begin
                alu_control <= 3'b000;  //add
            end
            6'b000101: begin
                alu_control <= 3'b000;  //add
            end
            6'b010100: begin
                alu_control <= 3'b001; //sub
            end
            6'b010101: begin
                alu_control <= 3'b001; //sub
            end
            6'b100000: begin
                alu_control <= 3'b000;  //add
            end
            6'b100001: begin 
                alu_control <= 3'b001; //sub
            end
            6'b100100: begin
                alu_control <= 3'b101; //set less than
            end
            6'b100101: begin
                alu_control <= 3'b101; //set less than
            end
            6'b101100: begin
                alu_control <= 3'b011; //or
            end
            6'b101101: begin
                alu_control <= 3'b011; //or
            end
            6'b101110: begin 
                alu_control <= 3'b010; //and
            end
            6'b101111: begin 
                alu_control <= 3'b010; //and
            end
        endcase
    end
endmodule

// PC Source
module PC_Src(pc_src, zero, branch);
  input zero, branch;
  output pc_src;
	assign pc_src = zero & branch;
endmodule


// Test benches
//module Mux_tb();
//  reg [31:0]I1, I2;
//  reg s;
//  wire [31:0]Out;
//  Mux tb(I1, I2, s, Out);
  
//  initial
 // begin
//    I1 = 320;
//    I2 = 1;
//    s = 0;
//    #10 s = 1;
//  end
//endmodule

module risc_v();
  reg clk, rst;
  RISC_V_Processor uut(clk, rst);
  
  initial
  clk = 1;
  
  always
  #10 clk = ~clk;
  
  initial
  begin
    rst = 1;
    #10 rst = 0;
    
//    $display("instruction: %h, PC: : %h", instruction, PC1);
  end
endmodule


