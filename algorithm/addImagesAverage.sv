// A file to add multiple images to calculate the average
// Author: Rebecca Rollins
module addImagesAverage (clk, reset, image, newImage, out);
	input logic clk;
	input logic reset;
	input logic [783:0][23:0] image;
	input logic [783:0][7:0] newImage;
	input logic [783:0][23:0] out;
	
	integer i;
	always_comb begin
		for(i = 0; i < 784; i = i + 1) begin
			out[i][23:0] = image[i][7:0] + newImage[i][7:0];
		end
	end
	
endmodule