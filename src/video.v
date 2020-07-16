`default_nettype none
module video (
  input             clk,
  input             reset,
  output [7:0]      vga_r,
  output [7:0]      vga_b,
  output [7:0]      vga_g,
  output            vga_hs,
  output            vga_vs,
  output            vga_de,
  input  [15:0]     vid_dout,
  output reg [14:1] vid_addr,
  input             mode
);

  parameter HA = 720;
  parameter HS  = 96;
  parameter HFP = 12;
  parameter HBP = 36;
  parameter HT  = HA + HS + HFP + HBP;

  parameter VA = 576;
  parameter VS  = 5;
  parameter VFP = 5;
  parameter VBP = 39;
  parameter VT  = VA + VS + VFP + VBP;
  parameter HBadj = 2;

  reg [7:0]  vb  = 32;
  reg [7:0]  hb  = 104;
  wire [7:0] hb2 = hb[6:1];
  wire [7:0] vb2 = vb[6:1];

  reg [9:0] hc = 0;
  reg [9:0] vc = 0;

  always @(posedge clk) begin
    if (reset) begin
      hc <= 0;
      vc <= 0;
    end else begin
      if (hc == HT - 1) begin
        hc <= 0;
        if (vc == VT - 1) vc <= 0;
        else vc <= vc + 1;
      end else hc <= hc + 1;
    end
  end

  assign vga_hs = !(hc >= HA + HFP && hc < HA + HFP + HS);
  assign vga_vs = !(vc >= VA + VFP && vc < VA + VFP + VS);
  assign vga_de = !(hc >= HA || vc >= VA);

  wire [7:0] x = (mode == 1 ? hc[9:1] - hb2 : hc - hb);
  wire [7:0] y = (mode == 1 ? vc[9:1] - vb2 : vc - vb);

  wire [7:0] x2 = x + 2;

  wire hBorder = (hc < (hb + HBadj) || hc >= HA - (hb + HBadj));
  wire vBorder = (vc < vb || vc >= VA - vb);
  wire border = hBorder || vBorder;

  // Read 2 pixels at a time
  reg [7:0] pixels0, pixels1;
  wire [3:0] pixel = mode == 1  ? {1'b0, pixels0[7], pixels1[7], pixels1[6]} 
                                : {1'b0, pixels0[7], pixels1[7], 1'b0};

  always @(posedge clk) begin
    if (mode == 1) begin
      if (hc[0] && hc < HA) begin
        if (x[1:0] == 2) vid_addr <=  {y, x2[7:2]};
        if (x[1:0] == 3) {pixels0, pixels1}  <= vid_dout;
        else begin
          pixels0 <= {pixels0[5:0],2'b0};
          pixels1 <= {pixels1[5:0],2'b0};
        end
      end
    end else begin
      if (hc < HA) begin
        if (x[2:0] == 6) vid_addr = {y, x2[7:3]};
        if (x[2:0] == 7) {pixels0, pixels1} = vid_dout;
	else begin
          pixels0 <= {pixels0[6:0],1'b0};
          pixels1 <= {pixels1[6:0],1'b0};
        end
      end
    end 
  end

  wire [7:0] green = border ? 8'b0 : pixel[2] ? 8'hff : 8'b0;
  wire [7:0] red   = border ? 8'b0 : pixel[1] ? 8'hff : 8'b0;
  wire [7:0] blue  = border ? 8'b0 : pixel[0] ? 8'hff : 8'b0;

  assign vga_r = !vga_de ? 8'b0 : red;
  assign vga_g = !vga_de ? 8'b0 : green;
  assign vga_b = !vga_de ? 8'b0 : blue;

endmodule

