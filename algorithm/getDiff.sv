// pixel-pixel difference
module getDiff (diff, pixel1, pixel2);
  input logic [7:0] pixel1, pixel2;
  output logic [15:0] diff; // 16 bits
  
  always_comb begin
    if(pixel1 > pixel2) begin
	   diff = (pixel1 - pixel2)**2;
    end else begin    
		diff = (pixel2 - pixel1)**2;
	 end
  end
endmodule 