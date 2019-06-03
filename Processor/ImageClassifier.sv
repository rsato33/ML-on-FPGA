module ImageClassifier(	CLOCK_50, KEY, VGA_R, VGA_G, 
								VGA_B, VGA_BLANK_N, VGA_CLK, VGA_HS, 
								VGA_SYNC_N, VGA_VS, GPIO_1, LEDR);
	input [3:0] KEY;
	output [9:0] LEDR;
	input CLOCK_50;
	output logic [7:0] VGA_R;
	output logic [7:0] VGA_G;
	output logic [7:0] VGA_B;
	output logic VGA_BLANK_N;
	output logic VGA_CLK;
	output logic VGA_HS;
	output logic VGA_SYNC_N;
	output logic VGA_VS;
	inout [35:0] GPIO_1;
	
	logic [31:0] imageRegister;
	logic [783:0][7:0] known, unknown;
	logic [31:0] distance;
	logic [2:0] piState;
	
	
	vgaImage vga (	known, 
						CLOCK_50, 
						CLOCK_50, 
						~KEY[0], //Pose edge reset
						VGA_R, 
						VGA_G, 
						VGA_B, 
						VGA_BLANK_N, 
						VGA_CLK, 
						VGA_HS, 
						VGA_SYNC_N, 
						VGA_VS);
	
	assign piState = {GPIO_1[7], GPIO_1[1], GPIO_1[3]};
						
	distance calDistance(.clk(GPIO_1[5]),
								.reset(KEY[1]),
								.state(piState),
								.unknown(unknown), 
								.known(known), 
								.distance(distance));

	ImageTransfer (.clk(GPIO_1[5]), 
						.reset(KEY[1]), //Different key to reset the transfer, negedge reset
						.dataRegister({GPIO_1[0], GPIO_1[2], GPIO_1[4], GPIO_1[6], GPIO_1[8], GPIO_1[10], GPIO_1[12], GPIO_1[14]}), 
						.storeState(piState),
						.toggleWrite(GPIO_1[9]),
						.known(known),
						.unknown(unknown),
						.distance(distance));
						

endmodule



// divided_clocks[0] = 25MHz, [1] = 12.5Mhz, ... [23] = 3Hz, [24] = 1.5Hz,
// [25] = 0.75Hz, ...
module clock_divider (clock, divided_clocks);
	input logic clock;
	output logic [31:0] divided_clocks;

	initial begin
		divided_clocks <= 0;
	end

	always_ff @(posedge clock) begin
		divided_clocks <= divided_clocks + 1;
	end

endmodule 
