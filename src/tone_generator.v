module tone_generator (
  input       clk,
  input [8:0] freq,
  input [7:0] fuzz,
  output      audio_out
);

  parameter CLK_HZ = 27000000;

  reg [32:0]  count;
  reg         tone;
  reg [7:0]   fuzz_reg;
  wire [32:0] limit = ((CLK_HZ / 2) * (freq + fuzz_reg));
   
  always @(posedge clk) begin
    if (freq != 0) begin
      if (count >= limit)  begin
        count <= 0;
        tone  <= !tone;
        fuzz_reg <= fuzz;
      end else begin
        count <= count + 11336;
      end
    end else begin
      count <= 0;
      tone <= 1'b0;
    end
  end

  assign audio_out = tone;

endmodule
