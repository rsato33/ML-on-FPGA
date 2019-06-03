// image-test distance
module addDiff (sum, pdiff);
  input logic [783:0][15:0] pdiff;
  output logic [31:0] sum;
  
  integer i;
  always_comb begin
    sum = 0;
    for (i = 0; i < 784; i = i + 1)
      sum = sum + pdiff[i];
  end
endmodule 