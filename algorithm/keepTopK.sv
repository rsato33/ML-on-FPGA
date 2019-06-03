// keep top K =5 smallest distance
module keepTopK #(parameter K=5)(distance, currTopK, newTopK);
  input logic [31:0] distance;
  input logic [K-1:0][31:0] currTopK;
  output logic [K-1:0][31:0] newTopK;
  
  logic [31:0] min;
  
  integer index;
  integer i;
  always_comb begin
    min = currTopK[0];
	 index = 0;
	 
	 for (i = 0; i < K; i = i + 1) begin
	    if (currTopK[i] < min) begin
		   min = currTopK[i];
			index = i;
		 end
	 end 
	 
	 if (distance < min)
		  currTopK[index] = distance;
	 
	 for (i = 0; i < K; i = i + 1)
      newTopK[i] = currTopK[i];
	 
  end    
endmodule  