module riscv_mmio_uart #(parameter BAUD_RATE=115200)(
  input clk,
  input rst,
  input tx_rx_start,		//enable for uart transmission and recieving
  output [7:0]uart_output_rx, //output data from UART rx
  output tx_done,		//pulled high when data transmitting is completed
  output rx_done		//pulled high when data recieving is completed
);
  wire [31:0]PCNext, PC, instr, PCPlus4, PCTarget, SrcA, RD2, ImmExt, SrcB, ALUResult, ReadData, Result;
  wire PCSrc, ResultSrc, MemWrite, ALUSrc, RegWrite, Zero;
  wire [2:0]ALUControl;
  wire [1:0]ImmSrc;
  wire [7:0]uart_reciever;  // from rx output
  wire  [7:0] uart_transmitter; //is used to transfer data to input of UART tx	
  //CALLING MUX 1 FOR PC NEXT
  Mux M1(.I1(PCPlus4), .I2(PCTarget), .s(PCSrc), .Out(PCNext));  

  // CALLING PROGRAM COUNTER
  Program_Counter P_C(.clk(clk), .PC_In(PCNext), .rst(rst), .PC_Out(PC)); 

  // CALLING ADDER 1  FOR PC PLUS 4
  Adder Add1(.a(PC), .b(32'd4), .out(PCPlus4)); 

 //CALLING INSTRUCTION MEMORY
  instruction_memory IM(.inst_addr(PC), .instr(instr));	

 // CALLING RESGISTER FILE
  registerFile RF(.WriteData(Result), .rs1(instr[19:15]), .rs2(instr[24:20]), .rd(instr[11:7]), .RegWrite(RegWrite), .clk(clk), .ReadData1(SrcA), .ReadData2(RD2));

 // CALLING MUX 2 FOR SRC B
  Mux M2(.I1(RD2), .I2(ImmExt), .s(ALUSrc), .Out(SrcB));

 // CALLING ALU
  ALU ALU1(.Result(ALUResult), .a(SrcA), .b(SrcB), .op(ALUControl), .zero(Zero));

 // CALLING MMIO WHICH IS INTEGERATED WITH UART ON ADDRESS 60
  mmio_to_UART #(.BAUD_RATE(BAUD_RATE)) mmio_uart(
	 .clk(clk),
	 .address(ALUResult),  
	 .WD(RD2),		
	 .uart_reciever(uart_reciever),
	 .mem_write(MemWrite),		
	 .uart_transmitter(uart_transmitter), 
	 .tx_rx_start(tx_rx_start),	
	 .RD(ReadData),	
	 .uart_output_rx(uart_output_rx),	
	 .tx_done(tx_done),	
	 .rx_done(rx_done)

 );
 //CALLING MUX 3 FOR RESULT
  Mux M3(.I1(ALUResult), .I2(ReadData), .s(ResultSrc), .Out(Result));
 // CALLING IMMEDIATE EXTENDER
  imm_ext IDE(.instruction(instr), .immSrc(ImmSrc), .imm_data(ImmExt));
 // CALLING ADDER 2 FOR PC TARGET
  Adder Add2(.a(PC), .b(ImmExt), .out(PCTarget));
// CALLING CONTROL UNIT
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


// Program Counter
module Program_Counter(clk, PC_In, rst, PC_Out);
  input clk, rst;
  input [31:0]PC_In;
  output reg [31:0]PC_Out;
  always @(posedge clk)
  begin
    if (rst==1)
      PC_Out <= 0;
    else if (rst==0)
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
   imem[0]= 8'b00100011; //23   
   imem[1]= 8'b10101110; //AE
   imem[2]= 8'b01100001; //61
   imem[3]= 8'b00000010; //02	// sw x6,60(x3)
	
   imem[4]= 8'b00000011; //03
   imem[5]= 8'b10100000; //a0
   imem[6]= 8'b11000001; //c1
   imem[7]= 8'b00000011; //03 	// lw x0,60(x3)

   imem[8]= 8'b00100011; //23
   imem[9]= 8'b00100100; //a4
   imem[10]= 8'b10100001; //64
   imem[11]= 8'b00000010; //00 	// sw x10,40(x2)

   imem[12]= 8'b10000011; //83
   imem[13]= 8'b00100010; //22
   imem[14]= 8'b10000001; //81
   imem[15]= 8'b00000010; //02	// lw x5,40(x2)
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
      2'b10: imm_data = {{21{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8]};
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

//MEMORY MAPPED IO INTEGERATED WITH UART MODULE AT IO DEVICE 1
module mmio_to_UART #(parameter BAUD_RATE=115200)(
	input clk,
	input [31:0] address,  // from risc v ALU output
	input [31:0] WD,		//input data from risc v which goes to UART
	input [7:0] uart_reciever,	// recieves data from UART rx output when reading
	input mem_write,		//control signal for enabling write	
	input  [7:0] uart_transmitter, //is used to transfer data to input of UART tx	
	input tx_rx_start,	//enable for uart transmission and recieving
	output  [31:0] RD,	// read data
	output [7:0]uart_output_rx,	//output data from UART rx
	output tx_done,	//pulled high when data transmitting is completed
	output rx_done	//pulled high when data recieving is completed

);
	wire tx;	
	
	//reg clk;		//simulation clk generator(should be removed when instantiating)
	//initial begin
	//clk<=0;
	//end
	//always begin
	//#1 clk=~clk;
	//end


	memory_mapped_IO mmio (			// memory mapped IO called
		.clk(clk),
		.address(address),
		.WD(WD),
		.uart_reciever(uart_output_rx),
		.mem_write(mem_write),	
		.RD(RD),
		.uart_transmitter(uart_transmitter)

	);

	uart_final #(.BAUD_RATE(BAUD_RATE)) uart (	//UART module called (tx and rx) (NOTE THAT tx and rx are connected)
		.clk(clk),
		.data_in(uart_transmitter),
		.tx_rx_start(tx_rx_start),
		.data_out(uart_output_rx),
		.tx_done(tx_done),
		.rx_done(rx_done)

	);

endmodule

//MEMORY MAPPED IO (DATA_MEMORY)
module memory_mapped_IO(	//FOR 1 IO DEVICE
	input clk,
	input [31:0] address,  // from risc v ALU output
	input [31:0] WD,	//input data from risc v which goes to UART
	input [7:0] uart_reciever,	// 8 bit data from UART rx output
	input mem_write,	//control signal for enabling write
	output reg [31:0] RD,	// read data
	output reg [7:0] uart_transmitter // 8 bit data input to IO Device UART tx 
);
	reg [7:0] memory [63:0];	//1x64 memory
	reg WE1; // write enable for I0 devices
	reg WEM; // write enable for rest of data memory
	reg [1:0] RDsel; // control signal for Read data mux
	
	
	initial begin	//intializing memory map adresses for IO devices		
		memory[63] <= 8'bxxxxxxxx ; // IO DEVICE 4 
		memory[62] <= 8'bxxxxxxxx ; // IO DEVICE 3
		memory[61] <= 8'bxxxxxxxx; // IO DEVICE 2
		memory[60] <= 8'bxxxxxxxx; // IO DEVICE 1 (UART)
	end
	
	//IO DEVICES
	always@(posedge clk) begin 
		if(WE1 == 1 ) begin
			{memory[63],memory[62],memory[61],memory[60]}<= WD; //writes the 32 bit data to the 4 memory map addresses for IO devices
			uart_transmitter<= memory[60]; // sending data to IO device 1(UART) tx
		end
		else if ( RDsel == 2'b01) begin
			memory[60]<=uart_reciever;		//recieving data from  IO device 1(UART) rx
			RD<= {memory[63],memory[62],memory[61],memory[60]}; // loading the data of IO map locations to processor
		end
	end
	
	// REST OF DATA MEMORY
	always@(posedge clk) begin  
		if (WEM == 1) begin
			{memory[address+3],memory[address+2],memory[address+1],memory[address]} <= WD;	//32 bit store data from risc v register file to four,8 bit memory locations 			
	
		end
		else if (RDsel == 2'b00) begin 
			RD <= {memory[address+3],memory[address+2],memory[address+1],memory[address]} ; // loading 32 bit data to RD from 4 differnent 1 byte mem locations
		end
		
	
	end

	//ADDRESS DECODER
	always@ (posedge clk) begin	
		
		if(address == 32'd64 ||address == 32'd63 ||address == 32'd62 ||address ==32'd61 ) begin // if any of the address is detected the // if statement will be executed
			if(mem_write == 1) begin		//check for write enable
				WEM<=0;
				RDsel<='x;
				WE1<=1; end
			else if ( mem_write == 0) begin
				RDsel<= 1;				
				WEM<=0;
				WE1<=0; end
			
		end
		
		else begin
			
			if(mem_write==1) begin
				WEM<=1;
				WE1<=0; 
				RDsel<='x;  end
			else if(mem_write ==0) begin
				RDsel<=0;
				WEM<=0;
				WE1<=0;
			end
		end
			


	end
endmodule

//UART MODULE(CONTAINS INSTANTIATION OF TX,RX,BAUD GEN)
module uart_final #(parameter BAUD_RATE=115200)(		//can can control the baud rate by assigning the desired value 
	input  clk,					//to parameter BAUD_RATE here we have taken 24000 as an example
	input [7:0] data_in,				//value
	input  wire tx_rx_start,
	output [7:0]data_out,
	output tx_done,
	output rx_done

	);
	
	wire clk_baud;
	wire tx;
	wire rx;
	
	baudgen #(.BAUD_RATE(BAUD_RATE)) gen (		//calling baud generator module
		.clk(clk),
		.tx_rx_start(tx_rx_start),
		.clk_baud(clk_baud)
	);


	uart_tx transmitter (				//calling transmitter module
		.data_in(data_in),
		.tx_rx_start(tx_rx_start),	
		.clk_baud(clk_baud),
		.tx_done(tx_done),
		.tx(tx)
	);

	uart_rx reciever (				// calling reciever module
		.rx(tx),
		.tx_rx_start(tx_rx_start),
		.clk_baud(clk_baud),
		.rx_done(rx_done),
		.data_out(data_out)
	
	);
	

endmodule

//UART TRANSMITTING MODULE
module uart_tx(										
	input [7:0] data_in, //input data 8 bit	
	input wire tx_rx_start, //enable
	input clk_baud, //from baud gen
	output reg tx_done,  //indiactor
	output reg tx   //output serial data	
	
);

	parameter IDLE=2'b00;  //0 =idle
	parameter START=2'b01; //1=start
	parameter TRANS=2'b10; //2=transmission
	parameter STOP=2'b11; // 3= stop
	reg [1:0] current_state_tx= IDLE;
	reg [10:0] buffer;   //data pack
	reg [3:0] count;
	reg parity_bit;
		
	always@(posedge clk_baud) begin: FSM			//finite state machine
	
	case(current_state_tx)
		IDLE: 
		begin
			
			if(tx_rx_start == 1'b1  )
			begin	
				current_state_tx <= START;		//proceeds to start state as the enable pin is pulled high
				tx <= 1 ;				//else stays idle
				tx_done <= 0;
				buffer <= 0;	//initializing buffer
				parity_bit <= ^data_in;  // parity bit generation
		 
			end
			else 
			begin
				tx_done <= 0;		
				current_state_tx <= IDLE;
		 
			end
		end

		START:
		begin
			tx <= 0 ;
			if(tx_rx_start)						// input data loads in data packet as enable is pulled high
			begin							// then proceeds to tansmission state
				buffer <= {1'b1,parity_bit,data_in,1'b0}; // loading of data in data packet 
				count <= 0;
				tx_done <= 0;
				current_state_tx <= TRANS;
			end
			else
			begin
				current_state_tx <= START;
				tx_done <= 0;		
				count <= 0;
			end

		end

		TRANS:
		begin
			if(count==9)				//the count increases as the input data gets loaded bit by bit into the
			begin					// the buffer/data packet then proceeds to stop state
				tx  <= buffer[0];
            			buffer   <= buffer>>1;
            			count    <= count+1;
           			current_state_tx <= STOP;
         		end
			else 
			begin
            			tx      <= buffer[0];
           			buffer <=  buffer>>1;
            			count     <= count+1;
          		end
		end

		STOP:						//tx_done/indiactor is pulled high and the state is set to IDLE
		begin						// where all the values are intialized and 1 is trasnmitted until
			tx   <= 1;				// enable is pulled high
			tx_done <=1;
			current_state_tx <= IDLE;
		end

		default: current_state_tx <= IDLE;
		
	endcase
	
	end

endmodule

//UART RECIEVING MODULE
module uart_rx(						
	input reg rx, //recieving data serial	
	input wire tx_rx_start,   //enable
	input clk_baud, //from baud gen
	output reg rx_done, //data sent indicator
	output reg [7:0] data_out   //output data 8bit

);

	parameter IDLE=3'b000;  //0=idle
	parameter START=3'b001; //1=start
	parameter RECIEVE=3'b010; //2=transmission
	parameter CHECK_SUM=3'b011;	 //3=check sum/parity check
	parameter STOP=3'b100; // 4= stop
	reg parity_bit;
	reg [2:0] current_state_rx= 3'b000;
	reg [8:0] buffer;  // 8 data 1 parity(data pack)
	reg [3:0] count;
	
	
	always@(posedge clk_baud) begin: FSM		//finite state machine
	
	case(current_state_rx)
		IDLE: 
		begin
		
			if(tx_rx_start == 1 )                //when enable pin is 1 will move to start state if not will stay idle
			begin			
				rx_done <= 0;
				buffer <= 0;
				count <= 0;
				current_state_rx <= START;	
		 
			end
			else 
			begin				
				rx_done <= 0;
				current_state_rx <= IDLE;
		 
			end
		end

		START:
		begin		 
			if(rx==1'b0)				// when  rx=0 is recieved current state will switch to recieve state if not then it will remain in start state
			begin
				current_state_rx <= RECIEVE;
				buffer <= 0;
				count <= 0;
				rx_done <= 0;
				
			end
			else
			begin
				current_state_rx <= START; 
				buffer <= 0;          
				rx_done <= 0;		
				count <= 0;
			end

		end

		RECIEVE:
		begin
			if( count == 8 )
			begin
				buffer <= {rx,buffer[8:1]};
           			current_state_rx <= CHECK_SUM;
         		end
			else 
			begin
            			buffer <= {rx,buffer[8:1]};				//the count will increase as the revcieving data gets concatinated with the
            			count  <= count+1;					//1st to 8th bit of the register leaving the 9th bit for the parity check
          		end
		end

		CHECK_SUM:
		begin
			parity_bit = ^buffer[8:1];					//if the parity matches the first 1st to 8th bit is sent to output
			if( rx == parity_bit )
			begin
				data_out <= buffer [8:1];
				current_state_rx <= STOP;
			end
			else
			begin								//if not then an undefied value is def into the register with the 9th bit being the
   				data_out <= 'x;						//changed parity bit	
         			buffer[8] <= rx;
          			current_state_rx <= STOP;					//bot scenarios proceed to stop state
			end
				
		end

		STOP:							//stop state is just for the indication of data receive confirmation
		begin
			rx_done <=1;							
			current_state_rx <= START ;	
										//as it pulls the rx_done high and then proceeds to IDLE state where all the values 
		end									//are intialized once again and as we know that until the new data is transmitted the 
									// the transmitter gives 1 as output due to which the code deosn't proceed to recieving state
		default: current_state_rx <= IDLE;
		
	endcase	
	
	end

endmodule


//BAUD GENERATOR
module baudgen #(parameter BAUD_RATE=115200)( 		//baud rate generator
	input  clk ,
	input wire tx_rx_start,
	output reg clk_baud
);
		
	parameter FREQ = 1000000; //1Mhz;
	parameter baud = FREQ/BAUD_RATE;
	reg [31:0] count = 0;

	always @(posedge clk ) 				//for every rising edge of input clock the value of count increases and
	begin						//as it reaches the value of baud the clk_baud becomes high ,then it resets
		if(tx_rx_start == 1'b0) begin		// as the value of count becomes greater
			count <= 0;
		end
		else if(count>=baud) begin 			
			count <= 0;
		end
		else begin
			count <= count + 1  ;
		end
		
	end
	
	always@(posedge clk) begin 		
		clk_baud = count == baud;
	end
	
endmodule

