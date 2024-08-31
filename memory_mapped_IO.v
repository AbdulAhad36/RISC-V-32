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
	//reg clk;

	//initial begin
	//clk<=0;
	//end
	//always begin
	//#10 clk=~clk;
	//end
	
	
	initial begin	//intializing memory map adresses for IO devices		
		memory[63] <= 8'bxxxxxxxx ; // IO DEVICE 4 
		memory[62] <= 8'bxxxxxxxx ; // IO DEVICE 3
		memory[61] <= 8'bxxxxxxxx; // IO DEVICE 2
		memory[60] <= 8'bxxxxxxxx; // IO DEVICE 1 (UART)
	end
	
	always@(posedge clk) begin //IO DEVICE
		if(WE1 == 1 ) begin
			{memory[63],memory[62],memory[61],memory[60]}<= WD; //writes the 32 bit data to the 4 memory map addresses for IO devices
			uart_transmitter<= memory[60]; // sending data to IO device 1(UART) tx
		end
		else if ( RDsel == 2'b01) begin
			memory[60]<=uart_reciever;		//recieving data from  IO device 1(UART) rx
			RD<= {memory[63],memory[62],memory[61],memory[60]};
		end
	end

	always@(posedge clk) begin // REST OF DATA MEMORY 
		if (WEM == 1) begin
			{memory[address+3],memory[address+2],memory[address+1],memory[address]} <= WD;	//32 bit store data from risc v register file to four,8 bit memory locations 			
	
		end
		else if (RDsel == 2'b00) begin 
			RD <= {memory[address+3],memory[address+2],memory[address+1],memory[address]} ; // loading 32 bit data to RD from 4 differnent 1 byte mem locations
		end
		
	
	end

	always@ (posedge clk) begin	//ADDRESS DECODER
		
		if(address == 32'd64 ||address == 32'd63 ||address == 32'd62 ||address ==32'd61 ) begin // if any of the address is detected the 												// if statement will be executed
			if(mem_write == 1) begin		//check for write enable
				WEM<=0;
				WE1<=1; end
			else if ( mem_write == 0) begin
				RDsel<= 1;				
				WEM<=0;
				WE1<=0; end
			
		end
		
		else begin
			RDsel<= 0;
			if(mem_write==1) begin
				WEM<=1;
				WE1<=0; end
			else if(mem_write ==0) begin
				RDsel<=0;
				WEM<=0;
				WE1<=0;
			end
		end
			


	end
endmodule
