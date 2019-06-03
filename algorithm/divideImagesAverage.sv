// A file to divide the sum of images by the number of images
// Author: Rebecca Rollins
module divideImagesAverage (clk, reset, image, out, numImages);
	input logic clk;
	input logic reset;
	input logic [783:0][23:0] image;
	input logic [13:0] numImages;
	output logic [783:0][7:0] out;
	
	integer i;
	always_comb begin
		for(i = 0; i < 784; i = i + 1) begin
			out[i][7:0] = image[i][23:0] / numImages[13:0];
		end
	end
	
endmodule