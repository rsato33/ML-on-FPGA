
module cpu (
	clk_clk,
	distance_pio_export,
	classification_pio_export,
	read_diff_pio_export,
	done_read_pio_export,
	reset_reset_n,
	sdram_clk_clk);	

	input		clk_clk;
	input	[31:0]	distance_pio_export;
	input	[7:0]	classification_pio_export;
	input	[7:0]	read_diff_pio_export;
	output	[7:0]	done_read_pio_export;
	input		reset_reset_n;
	output		sdram_clk_clk;
endmodule
