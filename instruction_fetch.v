// Instrution Fetch
module instruction_fetch(  instr);
  reg rst;
  output [31:0]instr;
  wire [31:0]PC_in;
  wire [31:0]PC_out;
  reg clk;
	initial begin 
		clk=0;rst=1;
		#10 rst=0; end 
	always begin 
		#5 clk=~clk; end

  Program_Counter P_C(.clk(clk), .PC_In(PC_in), .rst(rst), .PC_Out(PC_out));
  Adder Add(.a(PC_out), .b(32'd4), .out(PC_in));
  instruction_memory IM(.inst_addr(PC_out), .instr(instr));
endmodule 

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
