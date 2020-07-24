`default_nettype none
module ql
#(
  parameter c_slowdown    = 0, // CPU clock slowdown 2^n times (try 20-22)
  parameter c_lcd_hex     = 1, // SPI LCD HEX decoder
  parameter c_sdram       = 1, // 0: BRAM 32K,  1: SDRAM
  parameter c_vga_out     = 0, // 0: Just HDMI, 1: VGA and HDMI
  parameter c_diag        = 1, // 0: No LED diagnostcs, 1: LED diagnostics
  parameter c_acia_serial = 0, // 0: disabled, 1: ACIA serial
  parameter c_esp32_serial= 1, // 0: disabled, 1: ESP32 serial (micropython console)
  parameter c_mhz         = 27000000 // Clock speed of CPU clock
)
(
  input         clk25_mhz,
  // Buttons
  input   [6:0] btn,
  // Switches
  input   [3:0] sw,
  // HDMI
  output  [3:0] gpdi_dp,
  // Keyboard
  output        usb_fpga_pu_dp,
  output        usb_fpga_pu_dn,
  inout         ps2Clk,
  inout         ps2Data,
  // Audio
  output  [3:0] audio_l,
  output  [3:0] audio_r,
  // ESP32 passthru
  input         ftdi_txd,
  output        ftdi_rxd,
  input         wifi_txd,
  output        wifi_rxd,  // SPI from ESP32
  input         wifi_gpio16,
  input         wifi_gpio5,
  output        wifi_gpio0,

  inout         sd_clk, sd_cmd,
  inout   [3:0] sd_d,

  output        sdram_csn,  // chip select
  output        sdram_clk,  // clock to SDRAM
  output        sdram_cke,  // clock enable to SDRAM
  output        sdram_rasn, // SDRAM RAS
  output        sdram_casn, // SDRAM CAS
  output        sdram_wen,  // SDRAM write-enable
  output [12:0] sdram_a,    // SDRAM address bus
  output  [1:0] sdram_ba,   // SDRAM bank-address
  output  [1:0] sdram_dqm,  // byte select
  inout  [15:0] sdram_d,    // data bus to/from SDRAM

  inout  [27:0] gp,gn,
  // SPI display
  output        oled_csn,
  output        oled_clk,
  output        oled_mosi,
  output        oled_dc,
  output        oled_resn,
  // Leds
  output [7:0]  leds
);

  localparam c_gap_clk_count = $rtoi((c_mhz / 1000) * 5.0); // gap is 5ms
  
  localparam MDV_IDLE = 0;
  localparam MDV_GAP = 1;
  localparam MDV_READING = 2;
  localparam MDV_WRITING = 3;

  // ===============================================================
  // System Clock generation
  // ===============================================================
  wire clk_sdram_locked;
  wire [3:0] clocks;

  ecp5pll
  #(
      .in_hz( 25*1000000),
    .out0_hz(135*1000000),
    .out1_hz( 27*1000000),
    .out2_hz(135*1000000),                // SDRAM core
    .out3_hz(135*1000000), .out3_deg(180) // SDRAM chip 45-330:ok 0-30:not
  )
  ecp5pll_inst
  (
    .clk_i(clk25_mhz),
    .clk_o(clocks),
    .locked(clk_sdram_locked)
  );

  wire clk_hdmi  = clocks[0];
  wire clk_vga   = clocks[1];
  wire clk_cpu   = clocks[1];
  wire clk_sdram = clocks[2];

  // ===============================================================
  // Reset generation
  // ===============================================================
  reg [15:0] pwr_up_reset_counter = 0;
  wire       pwr_up_reset_n = &pwr_up_reset_counter;
  reg [7:0]  R_cpu_control = 4;   // SPI loader, initially HALT to

  always @(posedge clk_cpu) begin
     if (clk_sdram_locked && !pwr_up_reset_n)
       pwr_up_reset_counter <= pwr_up_reset_counter + 1;
  end

  wire reset = !pwr_up_reset_n || !btn[0] || R_cpu_control[0];

  // ===============================================================
  // Ulx3s-specific pin assignments
  // ===============================================================

  // Pull-ups for us2 connector
  assign usb_fpga_pu_dp = 1;
  assign usb_fpga_pu_dn = 1;

  wire acia_txd, acia_rxd;
  generate
    if(c_acia_serial)
    begin
      assign acia_rxd = ftdi_txd;
      assign ftdi_rxd = acia_txd;
    end
    if(c_esp32_serial)
    begin
      assign wifi_rxd = ftdi_txd;
      assign ftdi_rxd = wifi_txd;
    end
  endgenerate

  // ===============================================================
  // Optional VGA output
  // ===============================================================
  wire   [7:0]  red;
  wire   [7:0]  green;
  wire   [7:0]  blue;
  wire          hSync;
  wire          vSync;

  // Pinout for Digilent VGA Pmod. Change for other pmods.
  generate
    genvar i;
    if (c_vga_out) begin
      for(i = 0; i < 4; i = i+1) begin
        assign gn[10-i] = red[4+i];
        assign gn[3-i] = green[4+i];
        assign gp[10-i] = blue[4+i];
      end
      assign gp[2] = vSync;
      assign gp[3] = hSync;
    end
  endgenerate

  // ===============================================================
  // Diagnostic leds
  // ===============================================================
  reg [15:0] diag16;

  generate
    genvar i;
    if (c_diag) begin
      for(i = 0; i < 4; i = i+1) begin
        assign gn[17-i] = diag16[8+i];
        assign gp[17-i] = diag16[12+i];
        assign gn[24-i] = diag16[i];
        assign gp[24-i] = diag16[4+i];
      end
    end
  endgenerate

  // ===============================================================
  // 68000 CPU
  // ===============================================================
  reg  fx68_phi1;                // Phi 1 enable
  reg  fx68_phi2;                // Phi 2 enable (for slow cpu)
  wire cpu_rw;                   // Read = 1, Write = 0
  wire cpu_as_n;                 // Address strobe
  wire cpu_lds_n;                // Lower byte
  wire cpu_uds_n;                // Upper byte
  wire cpu_E;                    // Peripheral enable
  wire vma_n;                    // Valid memory address
  wire vpa_n;                    // Valid peripheral address
  wire cpu_fc0;                  // Processor state
  wire cpu_fc1;
  wire cpu_fc2;
  reg  berr_n = 1'b1;            // Bus error.
  wire cpu_reset_n_o;            // Reset output signal
  reg  dtack_n = !vpa_n;         // Data transfer ack (always ready)
  wire bg_n;                     // Bus grant
  reg  bgack_n = 1'b1;           // Bus grant ack
  reg  vsync_irq;
  reg  ipl0_n = 1'b1;            // Interrupt request signals
  reg  ipl1_n = !(vsync_irq | gap_irq);
  reg  ipl2_n = 1'b1;
  wire [15:0] ram_dout;
  wire [15:0] rom_dout;
  wire [15:0] vga_dout;
  wire [15:0] cpu_din;           // Data to CPU
  wire [15:0] cpu_dout;          // Data from CPU
  wire [23:1] cpu_a;             // Address
  wire halt_n = ~R_cpu_control[2]; // prevent running SDRAM junk code

  // QL-compatible ports
  wire timer_cs = !vma_n && cpu_a[6:2] == 0;                   // $18000 - $18003
  wire timer_rst_cs = !vma_n && cpu_a[6:1] == 0 && !cpu_uds_n; // $18000
  wire timer_adj_cs = !vma_n && cpu_a[6:1] == 0 && !cpu_lds_n; // $18001
  wire tctrl_cs = !vma_n && cpu_a[6:1] == 1 && !cpu_uds_n;     // $18002
  wire ipcwr_cs = !vma_n && cpu_a[6:1] == 1 && !cpu_lds_n;     // $18003
  wire ipcirq_cs = !vma_n && cpu_a[6:1] == 16;                 // $18020/21
  wire mdv_cs = !vma_n && cpu_a[6:1] == 17;                    // $18022/23
  wire tdata_cs = mdv_cs && !cpu_uds_n;                        // $18022
  wire display_cs = !vma_n && cpu_a[6:1] == 49;                // $18063
  // Non_QL ports
  wire acia_cs  = !vma_n && cpu_a[6:2] == 1;                   // $18005 and $18007
  wire audio_cs = !vma_n && cpu_a[6:1] == 4;                   // $18009
  wire keybd_cs = !vma_n && cpu_a[6:1] == 5;                   // $1800B

  wire [7:0]  acia_dout;
  wire [63:0] kbd_matrix;
  reg  [31:0] timer = 0;
  reg  [24:0] prescaler;
  reg         mode = 1;
  reg [15:0]  ipc_ret;
  reg [7:0]   tctrl = 0;
  reg [1:0]   mdv_state;
  reg [9:0]   mdv_addr = 0;
  wire [7:0]  mdv_dout;
  reg         mdv_gap = 0;
  reg         mdv_req = 0;
  reg         mdv_tx_empty = 0;
  reg         mdv_rx_ready = 1;
  reg [7:0]   mdv_sel = 0;
  reg [7:0]   mctrl = 0;
  reg         mdv_present = 1;
  wire        mdv_gap_present = !mdv_present | mdv_gap;
  reg [17:0]  gap_counter;
  reg         gap_irq = 0;

  wire [7:0]  ipc_status = {ipc_ret[15], 3'b0, mdv_gap_present, mdv_rx_ready,
                            mdv_tx_empty, 1'b0}; // ipc busy always false
  reg         ipc_irq = 1;
  wire [7:0]  irq_pending = {1'b0, (mdv_sel == 0), timer[0], 1'b0, vsync_irq, 1'b0, 1'b0, gap_irq};
  reg [2:0]   irq_mask;
  reg [4:0]   irq_ack;
  reg [3:0]   ipc_data;
  reg [1:0]   timer_byte;
  reg         r_vSync;
  reg [5:0]   bit_counter;
  reg [3:0]   ipc_cmd;
  reg [3:0]   ipc_state;
  reg [2:0]   key_row;
  reg [2:0]   key_col;
  reg         key_shift;
  reg         key_ctrl;
  reg         key_alt;
  reg         key_pressed;
  reg [6:0]   ipc_bits;
  reg [63:0]  ipc_sound;
  reg [9:0]   pitch;
  reg [14:0]  duration;
  reg [1:0]   audio_state;
  reg [10:0]  audio_scale;

  // Create a 1 second timer
  always @(posedge clk_cpu) begin
    prescaler <= prescaler + 1;
    if (prescaler == (c_mhz - 1)) begin
      prescaler <= 0;
      timer <= timer + 1;
    end
    if (timer_rst_cs && !cpu_rw) begin
      timer <= 0;
      timer_byte <= 0;
    end
    if (timer_adj_cs && !cpu_rw) begin
      timer[{timer_byte,3'b0} + 7 -: 8] <= cpu_dout[7:0];
      timer_byte <= timer_byte + 1;
    end
  end

  // Set the vsync interrupt
  always @(posedge clk_cpu) begin
    r_vSync <= vSync;
    if (reset || irq_ack[3]) vsync_irq <= 0;
    else if (!vSync && r_vSync) vsync_irq <= 1;
  end

  // Set the ipc interrupt
  always @(posedge clk_cpu) begin
    if (reset || irq_ack[1]) ipc_irq <= 0;
    else if (ps2_key[10]) ipc_irq <= 1;
  end

  // Set the gap interrupt
  always @(posedge clk_cpu) begin
    if (reset || irq_ack[0]) gap_irq <= 0;
    else if (mdv_gap && irq_mask[0]) gap_irq <= 1;
  end

  reg [9:0] addr_max = 0;

  // Microdrive reads and status writes
  reg r_mdv_rd_cs;
  always @(posedge clk_cpu) begin
    r_mdv_rd_cs <= cpu_rw && mdv_cs;
    if (mdv_state == MDV_READING) begin
      if (gap_counter == (c_gap_clk_count - 1)) begin
        mdv_state <= MDV_GAP;
        gap_counter <= 0;
        mdv_gap <= 1;
      end else begin
        gap_counter <= gap_counter + 1;
      end
    end else if (mdv_state == MDV_GAP) begin
      mdv_req <= gap_counter < 8;  // Request data
      if (gap_counter == (c_gap_clk_count -1)) begin
        mdv_state <= MDV_READING;
        mdv_gap <= 0;
	gap_counter <= 0;
	mdv_addr <= 0;
      end else begin
        gap_counter <= gap_counter +1;
      end
    end
    // Selecting mdv1 starts reading the microdrive data
    if (!cpu_rw && ipcirq_cs && !cpu_uds_n) begin
      mctrl <= cpu_dout[15:8];
      // Select the microdrive
      if (!cpu_dout[9] && mctrl[1]) begin
        mdv_sel <= {mdv_sel[6:0], cpu_dout[8]};
        gap_counter <= 0;
        if (cpu_dout[8]) begin
          mdv_state <= MDV_GAP;
          mdv_gap <= 1;
        end else begin
          mdv_state <= MDV_IDLE;
          mdv_gap <= 0;
        end
      end
    end
    // If we are reading header or data, advance address on cycle after read
    if (mdv_state == MDV_READING && !(cpu_rw && mdv_cs) && r_mdv_rd_cs) begin
      mdv_addr <= mdv_addr + 1;
      if (mdv_addr + 1 > addr_max) addr_max <= mdv_addr + 1;
      if (mdv_addr == 14) diag16 <= mdv_dout;
      gap_counter <= 0; // Reset gap timer on reads
    end
  end

  // Other I/O area writes
  reg r_ipcwr_cs;
  wire [3:0] ipc_shift = {ipc_data[2:0], cpu_dout[1]};
  wire [63:0] sound_shift = {ipc_sound[62:0], cpu_dout[1]};
  wire [14:0] beep_length ={sound_shift[22:16], sound_shift[31:24]}; 
  always @(posedge clk_cpu) begin
    if (reset) begin
      bit_counter <= 0;
      ipc_data <= 0;
      ipc_cmd <= 0;
      mode <= 1;
      irq_mask <= 0;
      ipc_state <= 0;
      key_pressed <= 0;
      audio_state <= 0;
    end else begin
      if (ps2_key[10] & ps2_key[9]) key_pressed <= 1;
      irq_ack <= 5'b0;
      r_ipcwr_cs <= ipcwr_cs;
      if (!cpu_rw) begin
        // Set the display mode 0 = 512x512, 1 = 256x256 -  $18063
        if (display_cs) mode = cpu_dout[3];
        // tctrl - $18002
        if (tctrl_cs) tctrl <= cpu_dout[15:8];
        // Set the irq data - $18021
        if (ipcirq_cs && !cpu_lds_n) {irq_mask, irq_ack} <= cpu_dout[7:0];
        // Set the ipc data  - $18003
        if (ipcwr_cs && !r_ipcwr_cs) begin
          ipc_data <= ipc_shift;
          if (!cpu_dout[0]) bit_counter <= bit_counter + 1; // Ignore initial write of 1
          if (ipc_state == 0 && bit_counter[1:0] == 3) begin
            ipc_cmd <= ipc_shift;
            case (ipc_shift)
              0:  begin end // init
              1:  begin     // get status
                    ipc_state <= 1;
                    ipc_bits <= 8;
                    bit_counter <= 0;
                    ipc_ret <= {7'b0, key_pressed, 8'b0};
                    key_pressed <= 0;
                  end
              2:  begin end // open ser1
              3:  begin end // open ser2
              4:  begin end // close ser1
              5:  begin end // close ser2
              6:  begin end // read ser1
              7:  begin end // read ser2
              8:  begin     // read key
                    ipc_state <= 1; 
                    ipc_bits <= 16;    
                    ipc_ret <= {4'b0001, 1'b0, key_shift, key_ctrl, key_alt, 2'b0, ~key_row, key_col};
                    bit_counter <= 0;
                  end
              9:  begin     // read key row
                    ipc_state <= 2; // Get the keyrow
                    ipc_bits <= 4;
                    bit_counter <= 0;
                  end
              10: begin // set sound
                    ipc_state <= 3;
                    ipc_bits <= 64;
                    bit_counter <= 0;
                  end 
              11: audio_state <= 0; // kill sound
              12: begin end // set ipl1
              13: begin end // set baud rate
              14: begin end // read random
              15: begin end // test - not used
            endcase
          end else if (ipc_state == 1) begin // Send return data to QDOS
            if (bit_counter != 0) ipc_ret <= {ipc_ret[14:0], 1'b0};
            if (bit_counter == (ipc_bits - 1)) begin
              ipc_state <= 0;
              bit_counter <= 0;
            end
          end else if (ipc_state == 2) begin // Get parameters
            if (bit_counter == (ipc_bits - 1)) begin
              ipc_state <= 1;
              bit_counter <= 0;
              if (ipc_cmd == 9) begin
                ipc_ret <= {kbd_matrix[{ipc_shift, 3'b111} -: 8], 8'b0};
                ipc_bits <= 8;
              end
            end
          end else if (ipc_state == 3) begin // Get sound parameters
            ipc_sound <= sound_shift;
            if (bit_counter == (ipc_bits - 1)) begin
              ipc_state <= 0;
              bit_counter <= 0;
              pitch <= 10'd256 - sound_shift[63:56];
              duration <= beep_length;
              audio_state <= (beep_length == 0 ? 2 : 1);
              audio_scale <= 0;
            end
          end
        end
      end
      if (audio_state == 1) begin
        audio_scale <= audio_scale + 1;
        if (audio_scale == 1943) begin // 70us units
          if (duration != 0) duration <= duration - 1;
          else audio_state <= 0;
          audio_scale <= 0;
        end
      end
    end
  end

  // Address 0x18000 to 0x1FFFF used for peripherals
  // Also set autovector for interrupts
  assign vpa_n = (!(cpu_a[17:15] == 3) | cpu_as_n) & !(cpu_fc0 & cpu_fc1 & cpu_fc2);

  generate
  if(c_sdram) // SDRAM as ROM and RAM, BRAM as video
  assign cpu_din = acia_cs           ? {8'd0, acia_dout} :
                   timer_cs          ? (cpu_a[1] ? timer[15:0] : timer[31:16]) : // $18000
                   keybd_cs          ? kbd_matrix[{cpu_a[9:7], 3'b0} + 7 -: 8] :
                   ipcirq_cs         ? {ipc_status, irq_pending} : // $18020/18021
                   mdv_cs            ? {mdv_dout, mdv_dout} :
                   cpu_a[19:18] == 0 ? ram_dout :
                                       0;
  else // BRAM all
  assign cpu_din = cpu_a[17:15] == 0 ? rom_dout :
                   cpu_a[17:15] == 4 ? vga_dout :
                   acia_cs           ? {8'd0, acia_dout} :
                   timer_cs          ? (cpu_a[1] ? timer[15:0] : timer[31:16]) :
                   keybd_cs          ? kbd_matrix[{cpu_a[9:7], 3'b0} + 7 -: 8] :
                   cpu_a[17:15] == 5 ? ram_dout :
                                       0;
  endgenerate

  // Generate phi1 and phi2 clock enables for the CPU
  generate
    if(c_slowdown)
    begin
      // Run 68k CPU SLOW
      reg [c_slowdown-1:0] delay_cnt;
      always @(posedge clk_cpu)
      begin
        fx68_phi1 <= delay_cnt == 0;
        fx68_phi2 <= delay_cnt == {1'b1,{(c_slowdown-1){1'b0}}};
        delay_cnt <= delay_cnt + 1;
      end
    end
    else // c_slowdown == 0, Run 68k CPU at 13.5 MHz
      always @(posedge clk_cpu)
      begin
        fx68_phi1 <= ~fx68_phi1;
        fx68_phi2 <=  fx68_phi1;
      end
  endgenerate

  fx68k fx68k (
    // input
    .clk( clk_cpu),
    .HALTn(halt_n),
    .extReset(reset),
    .pwrUp(!pwr_up_reset_n),
    .enPhi1(fx68_phi1),
    .enPhi2(fx68_phi2),

    // output
    .eRWn(cpu_rw),
    .ASn(cpu_as_n),
    .LDSn(cpu_lds_n),
    .UDSn(cpu_uds_n),
    .E(cpu_E),
    .VMAn(vma_n),
    .FC0(cpu_fc0),
    .FC1(cpu_fc1),
    .FC2(cpu_fc2),
    .BGn(bg_n),
    .oRESETn(cpu_reset_n_o),
    .oHALTEDn(),

    // input
    .DTACKn(dtack_n),
    .VPAn(vpa_n),
    .BERRn(berr_n),
    .BRn(~R_cpu_control[2]), // no bus request
    .BGACKn(1'b1),
    .IPL0n(ipl0_n),
    .IPL1n(ipl1_n),
    .IPL2n(ipl0_n), // ipl 0 and 2 tied together on the 68008

    // busses
    .iEdb(cpu_din),
    .oEdb(cpu_dout),
    .eab(cpu_a)
  );

  // ===============================================================
  // 6850 ACIA (uart)
  // ===============================================================
  reg baudclk; // 16 * 9600 = 153600 = 27Mhz/176
  reg [7:0] baudctr = 0;
  always @(posedge clk_cpu) begin
    baudctr <= baudctr + 1;
    baudclk <= (baudctr > 87);
    if(baudctr > 175) baudctr <= 0;
  end

  // 9600 8N1
  ACIA acia(
    .clk(clk_cpu),
    .reset(reset),
    //.cs(acia_cs),
    .cs(tctrl_cs | tdata_cs),
    .e_clk(cpu_E),
    .rw_n(cpu_rw),
    .rs(tdata_cs),
    //.data_in(cpu_dout[7:0]),
    .data_in(tctrl_cs ? 8'h01 : cpu_dout[15:8]), // Set output when anything is written to tctrl
    .data_out(acia_dout),
    .txclk(baudclk),
    .rxclk(baudclk),
    .txdata(acia_txd),
    .rxdata(acia_rxd),
    .cts_n(1'b0),
    .dcd_n(1'b0)
  );

  // ====================================================
  // Joystick for OSD control and games
  // ===============================================================
  reg [6:0] R_btn_joy;
  always @(posedge clk_cpu)
    R_btn_joy <= btn;

  // ===============================================================
  // SPI Slave for RAM and CPU control
  // ===============================================================
  wire [15:0] ram_do; // from SDRAM chip
  wire        spi_ram_wr, spi_ram_rd;
  wire [31:0] spi_ram_addr;
  wire  [7:0] spi_mdv_do;
  wire  [7:0] spi_ram_di = spi_ram_addr[31:24]==8'hD1 ? spi_mdv_do : (spi_ram_addr[0] ? ram_do[7:0] : ram_do[15:8]);
  wire  [7:0] spi_ram_do;

  assign sd_d[0] = 1'bz;
  assign sd_d[3] = 1'bz; // FPGA pin pullup sets SD card inactive at SPI bus

  wire spi_irq;
  spi_ram_btn
  #(
    .c_sclk_capable_pin(1'b0),
    .c_addr_bits(32)
  )
  spi_ram_btn_inst
  (
    .clk(clk_cpu),
    .csn(~wifi_gpio5),
    .sclk(wifi_gpio16),
    .mosi(sd_d[1]), // wifi_gpio4
    .miso(sd_d[2]), // wifi_gpio12
    .btn(R_btn_joy),
    .irq(spi_irq),
    .mdv_req(mdv_req),
    .mdv_req_type(8'h01),
    .wr(spi_ram_wr),
    .rd(spi_ram_rd),
    .addr(spi_ram_addr),
    .data_in(spi_ram_di),
    .data_out(spi_ram_do)
  );

  // Used for interrupt to ESP32
  assign wifi_gpio0 = ~spi_irq;

  reg [7:0] R_spi_ram_byte[0:1];
  reg R_spi_ram_wr;
  reg spi_ram_word_wr;
  always @(posedge clk_cpu)
  begin
    R_spi_ram_wr <= spi_ram_wr;
    if(spi_ram_wr == 1'b1)
    begin
      if(spi_ram_addr[31:24] == 8'hFF)
        R_cpu_control <= spi_ram_do;
      else
        R_spi_ram_byte[spi_ram_addr[0]] <= spi_ram_do;
      if(R_spi_ram_wr == 1'b0)
      begin
        if(spi_ram_addr[31:24] == 8'h00 && spi_ram_addr[0] == 1'b1)
          spi_ram_word_wr <= 1'b1;
      end
    end
    else
      spi_ram_word_wr <= 1'b0;
  end
  wire [15:0] ram_di = { R_spi_ram_byte[0], R_spi_ram_byte[1] }; // to SDRAM chip

  // ===============================================================
  // Microdrive Buffer 1KB
  // ===============================================================
  mdv_bram mdv_bram_i (
    .clk(clk_cpu),
    .we_a(spi_ram_wr && spi_ram_addr[31:24] == 8'hD1),
    .addr_a(spi_ram_addr[9:0]),
    .din_a(spi_ram_do),
    .dout_a(spi_mdv_do),
    .addr_b(mdv_addr), // from QL
    .dout_b(mdv_dout)  // to QL
  );

  // ===============================================================
  // SDRAM or BRAM for rom
  // ===============================================================
  generate
  if(c_sdram) begin

  wire we = spi_ram_word_wr;
  wire re = spi_ram_addr[31:24] == 8'h00 ? spi_ram_rd : 1'b0;
  assign rom_dout = ram_do;
  assign ram_dout = ram_do;
  sdram sdram_i (
    // cpu side
    .clk_in(clk_sdram),
    .rst (~clk_sdram_locked),
    .din (R_cpu_control[1] ? ram_di   : cpu_dout),
    .dout(ram_do),
    .addr(R_cpu_control[1] ? {1'b0, spi_ram_addr[23:1]} : {1'b0, cpu_a[23:1]}),
    .udsn(R_cpu_control[1] ? ~(we|re) : cpu_uds_n),
    .ldsn(R_cpu_control[1] ? ~(we|re) : cpu_lds_n),
    .asn (R_cpu_control[1] ? ~(we|re) : cpu_as_n),
    .rw  (R_cpu_control[1] ? ~we      : cpu_rw),

    // SDRAM side
    .sd_clk (sdram_clk),
    .sd_cke (sdram_cke),
    .sd_data(sdram_d),
    .sd_addr(sdram_a),
    .sd_dqm (sdram_dqm),
    .sd_ba  (sdram_ba),
    .sd_cs  (sdram_csn),
    .sd_we  (sdram_wen),
    .sd_ras (sdram_rasn),
    .sd_cas (sdram_casn)
  );
  end
  else
  begin
  gamerom #(.MEM_INIT_FILE("../roms/test.mem")) 
  rom32 (
    .clk(clk_cpu),
    .we_b(spi_ram_word_wr), // used by OSD
    .addr_b(spi_ram_addr[14:1]),
    .din_b(ram_di),
    .addr(R_cpu_control[1] ? spi_ram_addr[14:1] : cpu_a[14:1]),
    .dout(rom_dout)
  );
  assign ram_do = rom_dout; // for SPI to read back what is written
  // ===============================================================
  // Ram
  // ===============================================================
  ram ram32 (
    .clk(clk_cpu),
    .addr(cpu_a[14:1]),
    .din(cpu_dout),
    .we(!cpu_rw && cpu_a[17:15] == 5),
    .dout(ram_dout),
    .ub(!cpu_uds_n),
    .lb(!cpu_lds_n)
  );
  end
  endgenerate

  // ===============================================================
  // Keyboard (not yet implemented)
  // ===============================================================
  wire [10:0] ps2_key;

  // Get PS/2 keyboard events
  ps2 ps2_kbd
  (
    .clk(clk_cpu),
    .ps2_clk(ps2Clk),
    .ps2_data(ps2Data),
    .ps2_key(ps2_key)
  );

  keyboard keyboard (
    .reset(!pwr_up_reset_n),
    .clk(clk_cpu),
    .ps2_key(ps2_key),
    .js0({btn[1],btn[3],btn[4],btn[5],btn[6]}),
    .js1({btn[1],btn[3],btn[4],btn[5],btn[6]}),
    .matrix(kbd_matrix)
  );

  assign key_row = kbd_matrix[7:0]   ? 0 :
                   kbd_matrix[15:8]  ? 1 :
                   kbd_matrix[23:16] ? 2 :
                   kbd_matrix[31:24] ? 3 :
                   kbd_matrix[39:32] ? 4 :
                   kbd_matrix[47:40] ? 5 :
                   kbd_matrix[55:48] ? 6 : 7;

  wire [7:0]  col = kbd_matrix[{key_row, 3'b111} -: 8] & 
                    (key_row == 7 && kbd_matrix[58:56] != 0 && 
                     kbd_matrix[63:59] != 0 ? 8'hf8 : 8'hff);

  assign key_col = col[0] ? 0 :
                   col[1] ? 1 :
                   col[2] ? 2 :
                   col[3] ? 3 :
                   col[4] ? 4 :
                   col[5] ? 5 :
                   col[6] ? 6 : 7;


  assign key_shift = kbd_matrix[56];
  assign key_ctrl  = kbd_matrix[57];
  assign key_alt   = kbd_matrix[58];

  // ===============================================================
  // Video
  // ===============================================================
  wire [14:1] vid_addr; // Used by vdp
  wire [15:0] vid_dout; // Used by vdp
  wire        vga_wr = !cpu_rw && cpu_a[17:15] == 4;
  wire        vga_de;

  vram video_ram (
    .clk_a(clk_cpu),
    .addr_a(cpu_a[14:1]),
    .we_a(vga_wr),
    .din_a(cpu_dout),
    .ub_a(!cpu_uds_n),
    .lb_a(!cpu_lds_n),
    .dout_a(vga_dout),
    .clk_b(clk_vga),
    .addr_b(vid_addr),
    .dout_b(vid_dout)
  );

  video vga (
    .clk(clk_vga),
    .vga_r(red),
    .vga_g(green),
    .vga_b(blue),
    .vga_de(vga_de),
    .vga_hs(hSync),
    .vga_vs(vSync),
    .vid_addr(vid_addr),
    .vid_dout(vid_dout),
    .mode(mode)
  );

  // ===============================================================
  // SPI Slave for OSD display
  // ===============================================================
  wire [7:0] osd_vga_r, osd_vga_g, osd_vga_b;
  wire osd_vga_hsync, osd_vga_vsync, osd_vga_blank;
  spi_osd
  #(
    .c_start_x(62), .c_start_y(80),
    .c_chars_x(64), .c_chars_y(20),
    .c_init_on(0),
    .c_transparency(1),
    .c_char_file("osd.mem"),
    .c_font_file("font_bizcat8x16.mem")
  )
  spi_osd_inst
  (
    .clk_pixel(clk_vga), .clk_pixel_ena(1),
    .i_r(red  ),
    .i_g(green),
    .i_b(blue ),
    .i_hsync(~hSync), .i_vsync(~vSync), .i_blank(~vga_de),
    .i_csn(~wifi_gpio5), .i_sclk(wifi_gpio16), .i_mosi(sd_d[1]), // .o_miso(),
    .o_r(osd_vga_r), .o_g(osd_vga_g), .o_b(osd_vga_b),
    .o_hsync(osd_vga_hsync), .o_vsync(osd_vga_vsync), .o_blank(osd_vga_blank)
  );

  // ===============================================================
  // Convert VGA to HDMI
  // ===============================================================
  HDMI_out vga2dvid (
    .pixclk(clk_vga),
    .pixclk_x5(clk_hdmi),
    .red  (osd_vga_r),
    .green(osd_vga_g),
    .blue (osd_vga_b),
    .vde  (~osd_vga_blank),
    .hSync(~osd_vga_hsync),
    .vSync(~osd_vga_vsync),
    .gpdi_dp(gpdi_dp),
    .gpdi_dn()
  );

  // ===============================================================
  // Audio
  // ===============================================================
  wire [3:0] sound_ao;
  wire audio_out;
  //assign audio_l = sound_ao | {4{audio_out}};
  assign audio_l = {4{audio_out}};
  assign audio_r = audio_l;
  wire sound_ready;
  reg [10:0] div;

  always @(posedge clk_cpu) begin
    if (reset) div <= 0;
    else begin
      if (audio_state != 0) div <= div + 1;
      else div <= 0;
    end
  end

  sn76489 #(.AUDIO_RES(4),.DIVIDER(8)) audio (
    .clk(clk_cpu),
    .clk_en(fx68_phi1),
    .reset(!pwr_up_reset_n),
    .ce_n(1'b0),
    .we_n(!audio_cs),
    .ready(sound_ready),
    .d(cpu_dout[7:0]),
    .audio_out(sound_ao)
  );

  tone_generator #(0) tone (
    .clk(clk_cpu),
    .clk_div16_en(div[10]),
    .reset(reset),
    .freq(pitch),
    .audio_out(audio_out)
  );

  // ===============================================================
  // Diagnostic leds
  // ===============================================================
  assign leds = {ipc_cmd, ipl1_n, ipl0_n, vsync_irq, mode};

  generate
  if(c_lcd_hex)
  begin
  // SPI DISPLAY
  reg [127:0] R_display;
  // HEX decoder does printf("%16X\n%16X\n", R_display[63:0], R_display[127:64]);
  always @(posedge clk_cpu)
    R_display <= { mdv_dout, 6'b0, mdv_addr, // 2nd HEX row
                   1'b0, R_btn_joy, cpu_dout, cpu_din, cpu_a, 1'b0 }; // 1st HEX row

  parameter C_color_bits = 16;
  wire [7:0] x;
  wire [7:0] y;
  wire [C_color_bits-1:0] color;
  hex_decoder_v
  #(
    .c_data_len(128),
    .c_row_bits(4),
    .c_grid_6x8(1), // NOTE: TRELLIS needs -abc9 option to compile
    .c_font_file("hex_font.mem"),
    .c_color_bits(C_color_bits)
  )
  hex_decoder_v_inst
  (
    .clk(clk_hdmi),
    .data(R_display),
    .x(x[7:1]),
    .y(y[7:1]),
    .color(color)
  );

  // allow large combinatorial logic
  // to calculate color(x,y)
  wire next_pixel;
  reg [C_color_bits-1:0] R_color;
  always @(posedge clk_hdmi)
    if(next_pixel)
      R_color <= color;

  wire w_oled_csn;
  lcd_video
  #(
    .c_clk_mhz(125),
    .c_init_file("st7789_linit_xflip.mem"),
    .c_clk_phase(0),
    .c_clk_polarity(1),
    .c_init_size(38)
  )
  lcd_video_inst
  (
    .clk(clk_hdmi),
    .reset(R_btn_joy[5]),
    .x(x),
    .y(y),
    .next_pixel(next_pixel),
    .color(R_color),
    .spi_clk(oled_clk),
    .spi_mosi(oled_mosi),
    .spi_dc(oled_dc),
    .spi_resn(oled_resn),
    .spi_csn(w_oled_csn)
  );
  //assign oled_csn = w_oled_csn; // 8-pin ST7789: oled_csn is connected to CSn
  assign oled_csn = 1; // 7-pin ST7789: oled_csn is connected to BLK (backlight enable pin)
  end
  endgenerate

endmodule

