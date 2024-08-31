module IO_device(
	//input clk,
	input [31:0] address,  // from risc v register file
	input [31:0] WD,	//input data from risc v which goes to UART
	input mem_write,	
	output reg [31:0] RD,	// output data from I/0 memory to risc v
	output reg [7:0] data_in, // input data for UART transmitter
	input reg [7:0] data_out //output from uart rx
);
	reg [7:0] IO_memory [31:0];	//1 byte by 32 memory
	
	reg clk;
	initial begin
	clk<=0;
	end
	always begin
	#1 clk=~clk;
	end
	
	//intializing the IO_memory
	integer i ;
	initial begin
		for(i=0;i<32;i=i+1) begin
			IO_memory[i]<=0;  //intializing the memory to 0(all locations)
		end
	end

	always@(posedge clk) begin
		if (mem_write == 1'b1) begin
			{IO_memory[address+3],IO_memory[address+2],IO_memory[address+1],IO_memory[address]} <= WD;	//32 bit store data from risc v register file to four,8 bit memory locations 	
			#100
			data_in<= IO_memory[address];	   // <= 8 bit data transfered from I/O memory to UART tx input		
			#1000				// <= time required for the peripheral device to recieve data 
			data_in<= IO_memory[address+1];	// fromt the UART
			#1000
			data_in<= IO_memory[address+2];
			#1000
			data_in<= IO_memory[address+3];
	
		end
		else if (mem_write == 1'b0) begin 
			IO_memory[address] <= data_out;
			#1000
			IO_memory[address+1] <= data_out;
			#1000
			IO_memory[address+2] <= data_out;
			#1000
			IO_memory[address+3] <= data_out;
			#1000
			RD <= {IO_memory[address+3],IO_memory[address+2],IO_memory[address+1],IO_memory[address]} ;
		end
		

	end

endmodule