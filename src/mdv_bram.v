module mdv_bram (
  input            clk,
  input            we_a,
  input      [9:0] addr_a, addr_b,
  input      [7:0] din_a,
  output reg [7:0] dout_a, dout_b
);
  reg [7:0] ram [0:1023];
  always @(posedge clk) begin
    if (we_a) begin
      ram[addr_a] <= din_a;
    end
    dout_a <= ram[addr_a];
    dout_b <= ram[addr_b];
  end
endmodule
