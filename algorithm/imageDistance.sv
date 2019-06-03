//E-distance
module distance (clk, reset, state, unknown, known, distance);
	input clk, reset;
	input [783:0][7:0] known;
	input [2:0] state;
	input [783:0][7:0] unknown;
	output [31:0] distance; //26 bits
	
	logic [31:0] intenralDistance;
	
	integer select = 0;
	
	//Most of these are not used for this module
	parameter RESET = 3'b000;
	parameter KNOWN = 3'b001;
	parameter KNOWN_INC_DIFF = 3'b010;
	parameter INC_DIFF = 3'b011;
	parameter UNKNOWN = 3'b100;
	parameter WRITE = 3'b101;
	parameter RESET_DISTANCE = 3'b110;
	
	assign distance = intenralDistance;
	
	always_ff @(negedge reset or posedge clk) begin
		if(!reset) begin
			intenralDistance <= 0;
			select <= 0;
		end else begin
			if(state == KNOWN_INC_DIFF || state == INC_DIFF) begin
				if(known[select] > unknown[select]) begin
						intenralDistance <= intenralDistance + (known[select]-unknown[select])**2;
				end else begin
					intenralDistance <= intenralDistance + (unknown[select]-known[select])**2;
				end
				select <= select + 1;
			end else if(state == RESET_DISTANCE) begin
				intenralDistance <= 0;
				select <= 0;
			end
		end
	end
 
endmodule 