module ImageTransfer2(clk, reset, imageRegister, readEnable, PI_STATE, FPGA_STATE, image);

	input clk, reset;
	input [7:0] imageRegister;
	input readEnable;
	input [1:0] PI_STATE;
	output [1:0] FPGA_STATE;
	output [783:0][7:0] image;
	
	logic [783:0][7:0] buffer, image_internal; 
	logic doneReading = 0;
	integer index_pos; 
	
	typedef enum bit[1:0] {	IDLE=2'b11, 
									READING=2'b10, 
									DONE_READING_BLOCK=2'b01, 
									DONE_READING_ALL=2'b00} StateEnum;
	StateEnum ps, ns = IDLE;
	
	
	//Test image values 
	logic [7:0] valueA = 8'b11111111; //White
	logic [7:0] valueB = 8'b01111111;
	logic [7:0] valueC = 8'b00111111;
	logic [7:0] valueD = 8'b00011111;
	logic [7:0] valueE = 8'b00001111;
	logic [7:0] valueF = 8'b00000111;
	logic [7:0] valueG = 8'b00000000; //Black


	//Pi states
	parameter BUSY = 2'b00;
	parameter READY = 2'b01;
	parameter CONTINUE = 2'b11;
	parameter DONE = 2'b10;
	
	
	always_comb begin
		case(ps) 
			IDLE: begin
				if(readEnable) begin
					if(PI_STATE == READY)
						ns = READING;
					else 
						ns = IDLE;
				end else
					ns = IDLE;
				
			end
			READING: begin //Read in four block chunks
				if(doneReading == 1'b1)
					ns = DONE_READING_BLOCK;
				else
					ns = READING;
			end
			DONE_READING_BLOCK: begin
				if(readEnable) begin
					if(PI_STATE == CONTINUE)
						ns = IDLE;
					else if(PI_STATE == DONE)
						ns = DONE_READING_ALL;
					else
						ns = DONE_READING_BLOCK;
				end else
					ns = DONE_READING_BLOCK;
			end
			DONE_READING_ALL: begin
				ns = IDLE;
			end			
		endcase 
	
	end
	
	integer i, j;

	assign image = image_internal;
	assign FPGA_STATE = ps;
	
	always_ff @(negedge reset or posedge clk) begin
		if(!reset) begin
			ps <= IDLE;
			for (i = 0; i < 28; i++) begin
				for(j = 0; j < 28; j+=2) begin
					if(i % 2 == 0) begin
						image_internal[(i*28)+j][7:0] <= valueA;
						image_internal[(i*28)+j+1][7:0] <= valueG;
					end else begin
						image_internal[(i*28)+j][7:0] <= valueG;
						image_internal[(i*28)+j+1][7:0] <= valueA;
					end
				end
			end
			doneReading <= 1'b0;		
			index_pos <= 0;
		end else begin
			ps <= ns;
			if(ps == READING && doneReading != 1'b1) begin
				image_internal[index_pos] <= imageRegister;
				index_pos <= index_pos + 1;
				doneReading <= 1'b1;
			end else if(ps == DONE_READING_ALL) begin 
				//buffer <= image_internal;
				index_pos <= 0;
				doneReading <= 1'b0;
			end else 
				doneReading <= 1'b0;			
		end
	end
endmodule 