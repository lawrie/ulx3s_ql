module tone_generator (
  input       clk,
  input [8:0] freq,
  output      audio_out
);

  parameter CLK_HZ = 27000000;

  reg [31:0]  count;
  reg         tone;
  wire [31:0] limit = ((CLK_HZ / 2) * freq);
   
  always @(posedge clk) begin
    if (freq != 0) begin
      if (count >= limit)  begin
        count <= 0;
        tone  <= !tone;
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
