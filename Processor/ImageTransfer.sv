module ImageTransfer(clk, reset, dataRegister, storeState, toggleWrite, known, unknown, distance);

	input clk, reset, toggleWrite;
	input [31:0] distance;
	inout [7:0] dataRegister;
	input [2:0] storeState;
	
	output logic [783:0][7:0] known, unknown;
	logic [783:0][7:0] internalKnown, internalUnknown; 
	logic [7:0] dataRegister, imageRegister;
	integer pixelPos = 0;
	logic [1:0] distanceByte = 0;
	
	//Test image values 
	logic [7:0] valueA = 8'b11111111; //White
	/*
	logic [7:0] valueB = 8'b01111111;
	logic [7:0] valueC = 8'b00111111;
	logic [7:0] valueD = 8'b00011111;
	logic [7:0] valueE = 8'b00001111;
	logic [7:0] valueF = 8'b00000111;
	*/
	logic [7:0] valueG = 8'b00000000; //Black
	
	//Read types
	parameter RESET = 3'b000;
	parameter KNOWN = 3'b001;
	parameter KNOWN_INC_DIFF = 3'b010;
	parameter INC_DIFF = 3'b011;
	parameter UNKNOWN = 3'b100;
	parameter WRITE = 3'b101;
	parameter RESET_DISTANCE = 3'b110;
	
	always_comb begin
		if(distanceByte == 2'b00)
			dataRegister = toggleWrite ? distance[7:0] : 8'bzzzzzzzz;
		else if(distanceByte == 2'b01)
			dataRegister = toggleWrite ? distance[15:8] : 8'bzzzzzzzz;
		else if(distanceByte == 2'b10)
			dataRegister = toggleWrite ? distance[23:16] : 8'bzzzzzzzz;
		else
			dataRegister = toggleWrite ? distance[31:24] : 8'bzzzzzzzz;
	end
	
	assign imageRegister = dataRegister;
	
	assign known = internalKnown;
	assign unknown = internalUnknown;
	
	always_ff @(negedge reset or posedge clk) begin
		if(!reset) begin
			for (int i = 0; i < 28; i++) begin
				for(int j = 0; j < 28; j+=2) begin
					if(i % 2 == 0) begin
						internalKnown[(i*28)+j][7:0] <= valueA;
						internalKnown[(i*28)+j+1][7:0] <= valueG;
					end else begin
						internalKnown[(i*28)+j][7:0] <= valueG;
						internalKnown[(i*28)+j+1][7:0] <= valueA;
					end
				end
			end
			pixelPos <= 0;
			distanceByte <= 0;
		end else begin
			if(storeState == KNOWN || storeState == KNOWN_INC_DIFF) begin
				internalKnown[pixelPos] <= imageRegister;
				pixelPos <= pixelPos + 1;
			end else if(storeState == UNKNOWN) begin
				internalUnknown[pixelPos] <= imageRegister;
				pixelPos <= pixelPos + 1;
			end else if(storeState == WRITE) begin
				distanceByte <= distanceByte + 2'b01;
			end else if(storeState == RESET) begin //Reset
				pixelPos <= 0;
				distanceByte <= 0;
			end	
		end
	end
endmodule 