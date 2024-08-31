
module test_bench #(parameter BAUD_RATE=200000)();
  reg clk, rst;
  reg tx_rx_start;		//enable for uart transmission and recieving
  
  wire [7:0]uart_output_rx; //output data from UART rx
  wire tx_done;		//pulled high when data transmitting is completed
  wire rx_done;


 riscv_mmio_uart #(.BAUD_RATE(BAUD_RATE)) rmu(
 	clk,
 	rst,
 	tx_rx_start,		//enable for uart transmission and recieving s
  	uart_output_rx, 	//output data from UART rx
  	tx_done,		//pulled high when data transmitting is completed
 	rx_done		//pulled high when data recieving is completed
);
  
  initial
  clk = 1;		// clk time period = 10 ps
  
  always
  #5 clk = ~clk;
  
  initial
  begin	
	rst = 1; #50 tx_rx_start=1;
	#720 tx_rx_start=0;	// sw x6,60(x3)	 
	#10 rst=0 ;  
	#1 rst='x;	// lw x0,60(x3)

	
	#109 rst=0 ;  
	#1 rst='x;	// sw x10,40(x2)

	#109 rst=0 ;  
	#1 rst='x;	// lw x5,40(x2)

  end
endmodule

