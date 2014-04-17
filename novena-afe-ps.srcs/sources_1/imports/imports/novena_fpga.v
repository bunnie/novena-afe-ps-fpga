//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2013, Andrew "bunnie" Huang
//
// See the NOTICE file distributed with this work for additional 
// information regarding copyright ownership.  The copyright holder 
// licenses this file to you under the Apache License, Version 2.0 
// (the "License"); you may not use this file except in compliance
// with the License.  You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing,
// code distributed under the License is distributed on an
// "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied.  See the License for the
// specific language governing permissions and limitations
// under the License.
//////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module novena_fpga(
		   output wire       APOPTOSIS,
		   
//		   input wire AUD6_TFS,
//		   input wire AUD6_TXC,
//		   input wire AUD6_TXD,
//		   input wire AUD_MCLK,
//		   input wire AUD_MIC_CLK,
//		   input wire AUD_MIC_DAT,
		   
//		   input wire BATT_NRST,
//		   input wire BATT_REFLASH_ALRT,
		   
		   input wire CLK2_N,
		   input wire CLK2_P,
		   
//		   input wire DDC_SCL,
//		   input wire DDC_SDA,
		   
//		   output wire ECSPI3_MISO,
//		   input wire ECSPI3_MOSI,
//		   input wire ECSPI3_RDY,
//		   input wire ECSPI3_SCLK,
//		   input wire ECSPI3_SS2,
		   
		   input wire EIM_BCLK,
		   input wire [1:0] EIM_CS,
		   inout wire [15:0] EIM_DA,
		   input wire [18:16] EIM_A,
		   input wire EIM_LBA,
		   input wire EIM_OE,
		   input wire EIM_RW,
//		   input wire EIM_WAIT,
		   
		   output wire FPGA_LED2,
//		   input wire FPGA_LSPI_CLK,
//		   input wire FPGA_LSPI_CS,
//		   input wire FPGA_LSPI_HOLD,
//		   input wire FPGA_LSPI_MISO,
//		   input wire FPGA_LSPI_MOSI,
//		   input wire FPGA_LSPI_WP,
		   
		   input wire I2C3_SCL,
		   inout wire I2C3_SDA,
		   
//		   input wire SMB_SCL,
//		   input wire SMB_SDA,
		   
//		   input wire UART4_CTS,
//		   input wire UART4_RTS,
//		   input wire UART4_RXD,
//		   input wire UART4_TXD,
		   
		   // input wire UIM_CLK,
		   // input wire UIM_DATA,
		   // input wire UIM_PWR,
		   // input wire UIM_PWRON,
		   // input wire UIM_RESET,

		   inout wire [15:0] F_DDR3_D,
		   inout wire F_UDQS_N,
		   inout wire F_UDQS_P,
		   inout wire F_LDQS_N,
		   inout wire F_LDQS_P,
		   output wire F_UDM,
		   output wire F_LDM,
		   
		   output wire [2:0] F_BA,
		   output wire F_CAS_N,
		   output wire [13:0] F_DDR3_A,
		   output wire F_DDR3_CKE,
		   output wire F_DDR3_CK_N,
		   output wire F_DDR3_CK_P,
		   output wire F_DDR3_ODT,
		   output wire F_RAS_N,
		   output wire F_WE_N,
		   inout wire F_DDR3_RZQ,
		   inout wire F_DDR3_ZIO,
		   output wire F_DDR3_RST_N,

		   input wire [0:0] F_LVDS_CK_N,
		   input wire [0:0] F_LVDS_CK_P,

//		   output wire F_DX6,
//		   output wire F_DX7,
//		   output wire F_DX12,
//		   output wire F_DX13,
//		   output wire F_DX14,
//		   output wire F_DX15,
//		   input wire F_DX18,
		   output wire F_DX0,
		   output wire F_DX5,
		   inout wire F_DX4,

//		   input wire [0:0] F_LVDS_P,
//		   input wire [3:3] F_LVDS_P,
//		   input wire [8:8] F_LVDS_P,
//		   input wire [14:10] F_LVDS_P,
//		   input wire [0:0] F_LVDS_N,
//		   input wire [3:3] F_LVDS_N,
//		   input wire [8:7] F_LVDS_N,
//		   input wire [14:10] F_LVDS_N,
		   input wire [14:0] F_LVDS_P,
		   input wire [14:0] F_LVDS_N,
//		   input wire F_LVDS_PC,
//		   input wire F_LVDS_NC,

		   input wire RESETBMCU
	 );

   wire [15:0] 		      eim_dout;
   wire [15:0] 		      eim_din;
   wire 		      clk;   // free-runs at 50 MHz, unbuffered
   wire 		      clk50; // zero-delay, DLL version of above. Use this.
   wire 		      clk25; // halved of above
   wire 		      clk12p5; // quartered of above
   wire 		      clk100; // doubled-up version of the above. For time base applications.
   wire 		      bclk;  // NOTE: doesn't run until first CPU access to EIM; then free-runs at 133 MHz
   reg [23:0] 		      counter;
   
   wire 		      ddr3_dll_locked;
   wire 		      ddr3clk;
   
   wire 		      ddr3_calib_done; // to mcb

   wire 		      ddr3_p2_cmd_en;
   wire [2:0] 		      ddr3_p2_cmd_instr;
   wire [5:0] 		      ddr3_p2_cmd_bl;
   wire [29:0] 		      ddr3_p2_cmd_byte_addr;
   wire 		      ddr3_p2_cmd_empty; // from mcb
   wire 		      ddr3_p2_cmd_full;

   wire 		      ddr3_p2_wr_en; // to mcb
   wire [3:0] 		      ddr3_p2_wr_mask;
   wire [31:0] 		      ddr3_p2_wr_data;
   wire 		      ddr3_p2_wr_full; // from mcb
   wire 		      ddr3_p2_wr_empty;
   wire [6:0] 		      ddr3_p2_wr_count;
   wire 		      ddr3_p2_wr_underrun;
   wire 		      ddr3_p2_wr_error;
   wire 		      ddr3_p2_wr_pulse;
   wire 		      p2_wr_pulse_gate;

   wire 		      ddr3_p3_cmd_en; // to mcb
   wire [2:0] 		      ddr3_p3_cmd_instr;
   wire [5:0] 		      ddr3_p3_cmd_bl;
   wire [29:0] 		      ddr3_p3_cmd_byte_addr;
   wire 		      ddr3_p3_cmd_empty; // from mcb
   wire 		      ddr3_p3_cmd_full;

   wire 		      ddr3_p3_rd_en; // to mcb
   wire [31:0] 		      ddr3_p3_rd_data; // from mcb
   wire 		      ddr3_p3_rd_full;
   wire 		      ddr3_p3_rd_empty;
   wire [6:0] 		      ddr3_p3_rd_count;
   wire 		      ddr3_p3_rd_overflow;
   wire 		      ddr3_p3_rd_error;
   wire 		      ddr3_p3_rd_pulse;
   wire 		      p3_rd_pulse_gate;
   
   
   wire 		      reset;
   wire 		      dll_reset;
   wire 		      bclk_reset;

   wire 	      bclk_dll, bclk_div2_dll, bclk_div4_dll, bclk_locked;
   wire 	      bclk_early;

   ////////////////////////////////////
   ///// MASTER RESET
   ////////////////////////////////////
   
   sync_reset dll_res_sync( .glbl_reset(!RESETBMCU), .clk(clk), .reset(dll_reset) );
   sync_reset master_res_sync( .glbl_reset(!RESETBMCU), .clk(clk50), .reset(reset) );
   sync_reset bclk_res_sync( .glbl_reset(!RESETBMCU), .clk(bclk_dll), .reset(bclk_reset) );
   
   ////////////////////////////////////
   ///// BCLK DLL -- generate zero-delay clock plus slower versions for internal use
   ////////////////////////////////////
   wire 	      bclk_int_in, bclk_io_in;
   IBUFG   clkibufg (.I(EIM_BCLK), .O(bclk) );
   BUFG    bclk_dll_bufg(.I(bclk), .O(bclk_int_in) );
   
   bclk_dll bclk_dll_mod( .clk133in(bclk_int_in), .clk133(bclk_dll),
			  .RESET(reset), .LOCKED(bclk_locked));

   wire 	      i_reset, i_locked;
   wire 	      o_reset, o_locked;
   wire 	      bclk_i, bclk_o;
   wire 	      i_fbk_out, i_fbk_in;
   wire 	      o_fbk_out, o_fbk_in;
   
   dcm_delay bclk_i_dll( .clk133(bclk_int_in), .clk133out(bclk_i),
			  .CLKFB_IN(i_fbk_in), .CLKFB_OUT(i_fbk_out),
			  .RESET(i_reset), .LOCKED(i_locked));

   dcm_delay bclk_o_dll( .clk133(bclk_int_in), .clk133out(bclk_o),
			  .CLKFB_IN(o_fbk_in), .CLKFB_OUT(o_fbk_out),
			  .RESET(o_reset), .LOCKED(o_locked));
   
   // lock it to the input path
   BUFIO2FB bclk_o_fbk(.I(bclk_o), .O(o_fbk_in));
//   assign o_fbk_in = bclk_o;
//   BUFG bclk_io_fbk(.I(bclk_io), .O(io_fbk_in));
   
   assign i_fbk_in = bclk_i;

   ////////////////////////////////////
   ///// ADC clock DLL -- generate zero-delay clock plus slower versions for internal use
   ////////////////////////////////////
   wire 	      ADC_SCLK;
   wire 	      ADC_SDIO;
   wire 	      ADC_CSB;
   wire 	      ADC_DITHER;
   wire 	      ADC_SYNC;
   wire 	      ADC_PDWN;

   // map PCB layout names to functional pin names
   assign F_DX5 = ADC_SCLK;
   assign F_DX0 = ADC_CSB;
   assign F_LVDS_N[3] = ADC_DITHER;
   assign F_LVDS_N[7] = ADC_SYNC;
   assign F_LVDS_P[3] = ADC_PDWN;

   assign ADC_DITHER = 1'b0; // set control in SPI mode
   assign ADC_PDWN = 1'b0; // don't power down
   assign ADC_SYNC = 1'b0; // not using sync

   wire [7:0] 	      adc_wdata;
   wire [12:0] 	      adc_addr;
   wire [7:0] 	      adc_rdata;
   wire 	      adc_rd_wr_n;
   wire 	      adc_commit;
   wire 	      adc_sbus_busy;
   wire 	      adc_drv;
   wire 	      adc_sdata_in;
   
   wire [29:0] 	      sample_length;
   
   IOBUF #(.DRIVE(12), .SLEW("SLOW")) IOBUF_adc (.IO(F_DX4), .I(ADC_SDIO), .T(!adc_drv), .O(adc_sdata_in));
   
   //// ADC custom serial interface controller to program the ADC mode
   ad9265_serial adcserial(
			   .sclk(ADC_SCLK),
			   .sdata(ADC_SDIO),
			   .scs(ADC_CSB),
			   .sdata_in(adc_sdata_in),
			   .sdata_drv(adc_drv),
			   .w_data(adc_wdata),
			   .r_data(adc_rdata),
			   .addr(adc_addr),
			   .rd_wr_n(adc_rd_wr_n),
			   .commit(adc_commit),
			   .busy(adc_sbus_busy),
			   .clk12p5(clk12p5)
			   );

   ////////////////////////////////////
   ///// Input SERDES from ADC
   ////////////////////////////////////
   wire [15:0] 	      adc_data_p;
   wire [15:0] 	      adc_data_n;

   assign adc_data_p[7] = F_LVDS_P[8];
   assign adc_data_p[6] = F_LVDS_P[0];
   assign adc_data_p[5] = F_LVDS_P[10];
   assign adc_data_p[4] = F_LVDS_P[12];
//   assign adc_data_p[3] = F_LVDS_PC;
   assign adc_data_p[3] = F_LVDS_P[4]; // not right, but let's try for mapping
   assign adc_data_p[2] = F_LVDS_P[13];
   assign adc_data_p[1] = F_LVDS_P[11];
   assign adc_data_p[0] = F_LVDS_P[14];
   
   assign adc_data_n[7] = F_LVDS_N[8];
   assign adc_data_n[6] = F_LVDS_N[0];
   assign adc_data_n[5] = F_LVDS_N[10];
   assign adc_data_n[4] = F_LVDS_N[12];
//   assign adc_data_n[3] = F_LVDS_NC;
   assign adc_data_n[3] = F_LVDS_N[4]; // not right, but let's try for mapping 
   assign adc_data_n[2] = F_LVDS_N[13];
   assign adc_data_n[1] = F_LVDS_N[11];
   assign adc_data_n[0] = F_LVDS_N[14];
   
   wire 	      adc_reset;
   sync_reset adc_res_sync( .glbl_reset(!RESETBMCU), .clk(clk50), .reset(adc_reset) );
   wire [63:0] 	      adc_d;
   wire 	      adc_oclk;
   wire 	      adc_oclkx2;

   wire 	      adc_sample;
   reg [2:0] 	      adc_sample_s;
   wire 	      adc_sample_oclk;

   reg [29:0] 	      sample_count;
   
   always @(posedge adc_oclk) begin // bring adc_sample into oclk domain
      adc_sample_s[2:0] <= {adc_sample_s[1:0], adc_sample};
   end

   always @(posedge adc_oclk) begin
      if( !adc_sample_s[2] & adc_sample_s[1] ) begin
	 sample_count <= 30'b0;
      end else if( adc_sample_s[2] && (sample_count < sample_length) ) begin
	 sample_count <= sample_count + 30'b1;
      end else begin
	 sample_count <= sample_count;
      end
   end
   assign adc_sample_oclk = (sample_count < sample_length) && adc_sample_s[2];
   
   adc_rx adc_rx(.data_p(adc_data_p[7:0]),
		 .data_n(adc_data_n[7:0]),

		 .clk_p(F_LVDS_CK_P),
		 .clk_n(F_LVDS_CK_N),

		 .adc_dat(adc_d),

		 .oclk(adc_oclk),
		 .oclkx2(adc_oclkx2),

		 .reset(adc_reset)
		 );

   //// ADC to DDR3 interface
   // ddr3 port wires
   wire        adc_ddr3_cmd_en;  // ok
   wire [3:0]  adc_ddr3_cmd_instr;  // ok
   wire [5:0]  adc_ddr3_cmd_bl;  // ok
   wire [29:0] adc_ddr3_cmd_byte_addr;  // ok
   wire        adc_ddr3_cmd_empty; 
   wire        adc_ddr3_cmd_full; // ok
   wire        adc_ddr3_wr_en;  // ok
   wire [7:0]  adc_ddr3_wr_mask; // ok
   wire [63:0] adc_ddr3_wr_data; // ok
   wire        adc_ddr3_wr_full; // ok
   wire        adc_ddr3_wr_empty;  // ok
   wire [6:0]  adc_ddr3_wr_count;
   wire        adc_ddr3_wr_underrun; // ok
   wire        adc_ddr3_wr_error; // ok

   // local state
   reg [4:0] 	      ddr3_burstcnt;
   reg [29:0] 	      ddr3_adc_adr;
   
   assign adc_ddr3_cmd_instr = 4'b0000;
   assign adc_ddr3_cmd_byte_addr = ddr3_adc_adr[29:0];
   assign adc_ddr3_cmd_bl = 6'b01_1111; // always write 32 fifo entries, 0-indexed
   assign adc_ddr3_cmd_en = (ddr3_burstcnt == 5'b1_1111);

   // for x in range(63,-1,-1):
   //   print "adc_d[%d],adc_d[%d]," % ((x+8), x)
   // assign adc_ddr3_wr_data[63:0] = {adc_d[15],adc_d[7],
   // 				    adc_d[14],adc_d[6],
   // 				    adc_d[13],adc_d[5],
   // 				    adc_d[12],adc_d[4],
   // 				    adc_d[11],adc_d[3],
   // 				    adc_d[10],adc_d[2],
   // 				    adc_d[9],adc_d[1],
   // 				    adc_d[8],adc_d[0],
				    
   // 				    adc_d[31],adc_d[23],
   // 				    adc_d[30],adc_d[22],
   // 				    adc_d[29],adc_d[21],
   // 				    adc_d[28],adc_d[20],
   // 				    adc_d[27],adc_d[19],
   // 				    adc_d[26],adc_d[18],
   // 				    adc_d[25],adc_d[17],
   // 				    adc_d[24],adc_d[16],

   // 				    adc_d[47],adc_d[39],
   // 				    adc_d[46],adc_d[38],
   // 				    adc_d[45],adc_d[37],
   // 				    adc_d[44],adc_d[36],
   // 				    adc_d[43],adc_d[35],
   // 				    adc_d[42],adc_d[34],
   // 				    adc_d[41],adc_d[33],
   // 				    adc_d[40],adc_d[32],
				    
   // 				    adc_d[63],adc_d[55],
   // 				    adc_d[62],adc_d[54],
   // 				    adc_d[61],adc_d[53],
   // 				    adc_d[60],adc_d[52],
   // 				    adc_d[59],adc_d[51],
   // 				    adc_d[58],adc_d[50],
   // 				    adc_d[57],adc_d[49],
   // 				    adc_d[56],adc_d[48]
   // 				    };

   assign adc_ddr3_wr_data[63:0] = {
				    adc_d[59],adc_d[51],
				    adc_d[58],adc_d[50],
				    adc_d[57],adc_d[49],
				    adc_d[56],adc_d[48],

				    adc_d[63],adc_d[55],
				    adc_d[62],adc_d[54],
				    adc_d[61],adc_d[53],
				    adc_d[60],adc_d[52],

				    adc_d[43],adc_d[35],
				    adc_d[42],adc_d[34],
				    adc_d[41],adc_d[33],
				    adc_d[40],adc_d[32],

				    adc_d[47],adc_d[39],
				    adc_d[46],adc_d[38],
				    adc_d[45],adc_d[37],
				    adc_d[44],adc_d[36],

				    adc_d[27],adc_d[19],
				    adc_d[26],adc_d[18],
				    adc_d[25],adc_d[17],
				    adc_d[24],adc_d[16],

				    adc_d[31],adc_d[23],
				    adc_d[30],adc_d[22],
				    adc_d[29],adc_d[21],
				    adc_d[28],adc_d[20],

				    adc_d[11],adc_d[3],
				    adc_d[10],adc_d[2],
				    adc_d[9],adc_d[1],
				    adc_d[8],adc_d[0],
				    
				    adc_d[15],adc_d[7],
				    adc_d[14],adc_d[6],
				    adc_d[13],adc_d[5],
				    adc_d[12],adc_d[4]
				    };
   
   // write whenever sampling is commanded, always write groups of 32
   assign adc_ddr3_wr_en = (adc_sample_oclk || (ddr3_burstcnt != 5'b0));
   assign adc_ddr3_wr_mask = 8'b0; // don't mask any writes
   
//   always @(posedge adc_oclkx2) begin
   always @(posedge adc_oclk) begin
      if( adc_sample_oclk || (ddr3_burstcnt != 5'b0) ) begin
	 ddr3_burstcnt <= ddr3_burstcnt + 5'b1;
      end else begin
	 ddr3_burstcnt <= 5'b0;
      end

      // this line below means every time sample is toggled, write pointer goes to 0 automatically
      if(!adc_ddr3_wr_en) begin // when we're not writing, reset pointer to 0
	 ddr3_adc_adr <= 30'b0; // reset buffer to zero when sampling is turned off, and buffer committed
	 // change the above constant if we want to do multiple offsets in DDR3
      end else begin
	 if(adc_ddr3_cmd_en) begin
	    ddr3_adc_adr <= ddr3_adc_adr + 30'h100;
	 end else begin
	    ddr3_adc_adr <= ddr3_adc_adr;
	 end
      end
   end // always @ (posedge adc_oclk)

   wire adc_samp_err_reset;
   reg [1:0] adc_samp_err_reset_s;
   reg adc_samp_cmd_over_err;
   reg adc_samp_data_full_err;
   reg adc_samp_data_under_err;
   reg adc_samp_data_general_err;

   // log error conditions for feedback
//   always @(posedge adc_oclkx2) begin
   always @(posedge adc_oclk) begin
      adc_samp_err_reset_s[1:0] = {adc_samp_err_reset_s[0], adc_samp_err_reset};

      if( adc_samp_err_reset_s[1] ) begin
	 adc_samp_cmd_over_err <= 1'b0;
      end else if( adc_ddr3_cmd_full ) begin
	 adc_samp_cmd_over_err <= 1'b1;
      end else begin
	 adc_samp_cmd_over_err <= adc_samp_cmd_over_err;
      end

      if( adc_samp_err_reset_s[1] ) begin
	 adc_samp_data_full_err <= 1'b0;
      end else if( adc_ddr3_wr_full ) begin
	 adc_samp_data_full_err <= 1'b1;
      end else begin
	 adc_samp_data_full_err <= adc_samp_data_full_err;
      end

      if( adc_samp_err_reset_s[1] ) begin
	 adc_samp_data_under_err <= 1'b0;
      end else if( adc_ddr3_wr_underrun ) begin
	 adc_samp_data_under_err <= 1'b1;
      end else begin
	 adc_samp_data_under_err <= adc_samp_data_under_err;
      end

      if( adc_samp_err_reset_s[1] ) begin
	 adc_samp_data_general_err <= 1'b0;
      end else if( adc_ddr3_wr_error ) begin
	 adc_samp_data_general_err <= 1'b1;
      end else begin
	 adc_samp_data_general_err <= adc_samp_data_general_err;
      end
   end // always @ (posedge adc_oclk)
   
      
   ////////////////////////////////////
   ///// I2C register set -- for aiding debug of EIM
   ////////////////////////////////////
   wire [7:0] 	      reg_0_test;
   wire       SDA_pd;
   wire       SDA_int;
   wire [7:0] eim_mon_ctl;
   wire       eim_mon_adv; // advance the monitor FIFO when this *rises*
   wire [7:0] eim_mon_stat;
   wire [23:0] mon_data;
   
   IOBUF #(.DRIVE(8), .SLEW("SLOW")) IOBUF_sda (.IO(I2C3_SDA), .I(1'b0), .T(!SDA_pd), .O(SDA_int));
   i2c_slave i2c_slave(
		       .SCL(I2C3_SCL),
		       .SDA(SDA_int),
		       .SDA_pd(SDA_pd),
		       .clk(clk25), // nominally 26 MHz, this is close enough
		       .glbl_reset(reset),
		       .i2c_device_addr(8'h3C),

		       .reg_46_trig(eim_mon_adv),
		       
		       // outputs from I2C block (CPU->FPGA) 0-3F
		       .reg_0(reg_0_test),

		       .reg_2(eim_mon_ctl[7:0]),
		       // bit 0:  enable trigger condition
		       // bit 1:  reset log

		       // ADC control block
		       .reg_4(adc_wdata[7:0]),  
		       .reg_5(adc_addr[7:0]),
		       .reg_6({adc_commit,adc_rd_wr_n,adc_addr[12:8]}),

		       // inputs to I2C block (FPGA->CPU) 40-7F
		       .reg_40(reg_0_test),
		       .reg_41(eim_mon_stat[7:0]),
		       // bit 0: empty
		       // bit 1: full

		       // ADC readback register from serial interface
		       .reg_42(adc_rdata),
		       .reg_43({7'b0,adc_sbus_busy}),

		       // EIM monitor data going back to CPU
		       // it auto-incs thanks to reg_46_trig
		       .reg_44(mon_data[7:0]),  
		       .reg_45(mon_data[15:8]),
		       .reg_46({mon_data[23:16]}),

		       // ID / version code
		       // minor / major
		       .reg_fc(8'h00), .reg_fd(8'h03), .reg_fe(8'h00), .reg_ff(8'h04)
		       );
      
   ////////////////////////////////////
   ///// Register set -- area-inefficient, high fan-out/in registers for controlling/monitoring internal signals
   ///// All registers split into write or read only blanks
   ///// 0x40000 - 0x40FFF is reserved for w/o
   ///// 0x41000 - 0x41FFF is reserved for r/o
   /////   -> if you want to check a w/o value, loop it back to an r/o register
   ////////////////////////////////////
   
   reg 		      cs0_r, rw_r;
   reg [15:0] 	      din_r;
   reg [18:0] 	      bus_addr_r;
   reg 		      adv_r;

   reg 		      cs0_in, rw_in, adv_in, cs1_in;
   reg [15:0] 	      din_in;
   reg [2:0] 	      a_in;
   reg 		      oe_in;

   wire 	      rbk_clear_error;
   wire 	      rbk_full_error;
   wire 	      rbk_empty_error;
   wire 	      rbk_enable;
   wire 	      rbk_init;
   wire [17:0] 	      rbk_startpage;
   wire [15:0] 	      rbk_burstlen;
   wire [15:0] 	      measured_burst;

   always @(posedge bclk_i) begin
      cs0_in <= EIM_CS[0];
      rw_in <= EIM_RW;
      din_in <= eim_din;
      adv_in <= !EIM_LBA; // latch address on LBA low
      a_in <= EIM_A[18:16];
      cs1_in <= EIM_CS[1];
      oe_in <= EIM_OE;

      cs0_r <= cs0_in;
      rw_r <= rw_in;
      din_r <= din_in;
      adv_r <= adv_in;
   end
   
   always @(posedge bclk_i) begin 
      if( adv_in ) begin
	 bus_addr_r <= {a_in, din_in};
      end else begin
	 bus_addr_r <= bus_addr_r;
      end
   end

   wire [15:0] r40000wo;
   wire [15:0] r40002wo;

   wire [15:0] ro_d;

   //////// write-only registers
   reg_wo reg_wo_40000 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40000),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( r40000wo[15:0] ) );
   
   reg_wo reg_wo_40002 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40002),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(1'b0), .rbk_d(ro_d), // unreadable
			 .reg_d( r40002wo[15:0] ) );

   // CPU->DDR3 raw iron write control register interface
   wire [1:0]  dummy_40020;
   reg_wo reg_wo_40020 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40020),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( {p2_wr_pulse_gate, dummy_40020[1:0], ddr3_p2_cmd_bl[5:0], 
				  ddr3_p2_cmd_en, ddr3_p2_cmd_instr[2:0] } ) );

   reg_wo reg_wo_40022 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40022),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( ddr3_p2_cmd_byte_addr[15:0] ) );

   reg_wo reg_wo_40024 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40024),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( ddr3_p2_cmd_byte_addr[29:16] ) );

   reg_wo reg_wo_40026 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40026),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( {ddr3_p2_wr_en, ddr3_p2_wr_mask[3:0]} ) );

   reg_wo reg_wo_40028 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40028),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( ddr3_p2_wr_data[15:0] ) );

   reg_wo reg_wo_4002A ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h4002A),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( ddr3_p2_wr_data[31:16] ) );
   
   reg_r_det reg_det_4102A (.clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h4002A),
			 .ena(!cs0_r && !rw_r),
			 .pulse( ddr3_p2_wr_pulse ) );

   
   // DDR3->CPU raw iron read control
   wire        burst_mode;  // 1 = read from burst interface; 0 = read via this interface
   wire [3:0]  reg_40030_dummy;
   reg_wo reg_wo_40030 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40030),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( {burst_mode, reg_40030_dummy[3:2], p3_rd_pulse_gate, 
				  reg_40030_dummy[1:0], ddr3_p3_cmd_bl[5:0], 
				  ddr3_p3_cmd_en, ddr3_p3_cmd_instr[2:0] } ) );

   reg_wo reg_wo_40032 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40032),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( ddr3_p3_cmd_byte_addr[15:0] ) );

   reg_wo reg_wo_40034 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40034),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( ddr3_p3_cmd_byte_addr[29:16] ) );


   wire [3:0]        ddr3_p3_dummy;
   reg_wo reg_wo_40036 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40036),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( {ddr3_p3_rd_en, ddr3_p3_dummy[3:0]} ) );


   // sampler control registers
   reg_wo reg_wo_40100 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40100),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( {adc_samp_err_reset, // reset error flags
				  adc_sample} ) );    // start sampling run (enhance with trigger eventually)

   reg_wo reg_wo_40102 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40102),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( sample_length[15:0] ) ); // # of samples to capture (LSB)

   reg_wo reg_wo_40104 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40104),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( sample_length[29:16] ) ); // MSB of above
   
   // readback control registers
   reg_wo reg_wo_40110 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40110),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( {rbk_clear_error,  // clear latched error flags
				  rbk_init,         // load the readback start page, flush fifos
				  rbk_enable} ) );  // begin readback

   reg_wo reg_wo_40112 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40112),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( rbk_startpage[15:0] ) ); // LSB of starting address, 4k-page 
   
   reg_wo reg_wo_40114 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40114),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( rbk_startpage[17:16] ) ); // MSB of above

   reg_wo reg_wo_40116 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40116),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( rbk_burstlen[15:0] ) ); // burst length of a readback
   
   //////// read-only registers
   // loopback readback
   reg_ro reg_ro_41000 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41000),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( r40000wo[15:0] ) );

   reg_ro reg_ro_41002 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41002),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( r40002wo[15:0] ) );

   // general status register
   // 0    : DDR3 DLL lock
   // 1    : DDR3 calibration done
   // 15-2 : reads as 0
   reg_ro reg_ro_41004 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41004),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {ddr3_calib_done, ddr3_dll_locked} ) );

   /////// ddr p2 write status
   reg_ro reg_ro_41020 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41020),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {ddr3_p2_wr_count[6:0],
				  2'b00,
				  ddr3_p2_cmd_empty, ddr3_p2_cmd_full,
				  ddr3_p2_wr_full, ddr3_p2_wr_empty, 
				  ddr3_p2_wr_underrun, ddr3_p2_wr_error} ) );

   /////// ddr p3 read status & data
   reg_ro reg_ro_41030 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41030),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {ddr3_p3_rd_count[6:0],
				  2'b00,
				  ddr3_p3_cmd_empty, ddr3_p3_cmd_full,
				  ddr3_p3_rd_full, ddr3_p3_rd_empty, 
				  ddr3_p3_rd_overflow, ddr3_p3_rd_error} ) );

   reg_ro reg_ro_41032 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41032),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( ddr3_p3_rd_data[15:0] ) );
   
   reg_ro reg_ro_41034 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41034),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( ddr3_p3_rd_data[31:16] ) );

   reg_r_det reg_det_41034 (.clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41034),
			 .ena(!cs0_r && rw_r),
			 .pulse( ddr3_p3_rd_pulse ) );

   // sample buffer status register
   reg_ro reg_ro_41100 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41100),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {12'b0,
				 adc_samp_cmd_over_err, 
				 adc_samp_data_full_err,
				 adc_samp_data_under_err,
				 adc_samp_data_general_err} ) );

   reg_ro reg_ro_41102 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41102),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {14'b0,
				  rbk_full_error,
				  rbk_empty_error} ) );

   reg_ro reg_ro_41104 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41104),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( measured_burst[15:0] ) );
   

   ///////////////////////
   ///////////////////////
   // CS1 bank registers: minimum size here is 64-bit, tuned for synchronous burst access only
   ///////////////////////
   ///////////////////////

   wire [63:0] 	     rC04_0000wo;
   wire [63:0] 	     rC04_0008wo;
   wire [15:0] 	     ro_d_b;
   
   ///////// write registers
   // loopback test
   reg_wo_4burst reg_wo_4b_C04_0000( .clk(bclk_i), .bus_ad(eim_din), .my_a(19'h4_0000), 
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]), 
				     .reg_d( rC04_0000wo[63:0] ), .rbk_d(ro_d_b) );

   reg_wo_4burst reg_wo_4b_C04_0008( .clk(bclk_i), .bus_ad(eim_din), .my_a(19'h4_0008),
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
				     .reg_d( rC04_0008wo[63:0] ), .rbk_d(ro_d_b) );


   ///////// read registers
   // loopback test
   reg_ro_4burst reg_ro_4b_C04_1000( .clk(bclk_i), .bus_ad(eim_din), .my_a(19'h4_1000),
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
				     .reg_d( rC04_0000wo[63:0] ), .rbk_d(ro_d_b) );

   reg_ro_4burst reg_ro_4b_C04_1008( .clk(bclk_i), .bus_ad(eim_din), .my_a(19'h4_1008),
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
				     .reg_d( rC04_0008wo[63:0] ), .rbk_d(ro_d_b) );
			 


   /// note to self: also rev version code for I2C readout side
   // FPGA minor version code
   reg_ro reg_ro_41FFC ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41FFC),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( 16'h0003 ) );

   // FPGA major version code
   reg_ro reg_ro_41FFE ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41FFE),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( 16'h0004 ) );

   ////////// VERSION LOG (major version 0004) /////////////
   //////
   // Minor version 0003, April 17 2014
   //    fix byte order to avoid swapping on host
   //
   //////
   // Minor version 0002, April 17 2014
   //    fix bit swapping due to e/o muxing on DDR, etc.
   //
   //////
   // Minor version 0001, April 15 2014
   //    retrofit to work with 16-bit ADC AD9265
   //
   ////////// VERSION LOG (major version 0003) /////////////
   //////
   // Minor version 0008, February 22 2014
   //    memory-to-EIM burst path finally doing something reasonable. 
   //    Added burst counters/burst length options
   //
   //////
   // Minor version 0007, February 19 2014
   //    fixing boogs. lots of minor bugs related to DDR3-to-memory burst path
   //
   //////
   // Minor version 0006, February 18 2014
   //    added DDR3-to-memory burst path
   //
   //////
   // Minor version 0005, February 16 2014
   //    added ADC-to-DDR3 path
   //
   //////
   // Minor version 0004, February 12 2014
   //    added HS ADC custom serial protocol layer
   //
   //////
   // Minor version 0003, February 11 2014
   //    Move to novena-fpga-afe major version 3
   //
   //////
   // Minor version 0002 
   //    bug fixes to timing & routing; fixed trigger fifo overflow timing error
   //
   //////
   // Minor version 0001, Dec 30 2013
   //    initial version; test FPGA for SDMA debugging
   //
   

   //////////////
   //////////////
   /// EIM bus monitor. "logic-analyzer" style interface to debug CS1-related timing issues
   //////////////
   //////////////

   eim_debug eim_debug(
		       .bclk_i(bclk_i),
		       .bclk_dll(bclk_dll),
		       .adv_in(adv_in),
		       .rw_in(rw_in),
		       .cs1_in(cs1_in),
		       .a_in(a_in),
		       .din_in(din_in),
		       .oe_in(oe_in),
		       .eim_trig_ena(eim_mon_ctl[0]),
		       .eim_mon_reset(eim_mon_ctl[1]),
		       .eim_mon_adv(eim_mon_adv),
		       .mon_empty(eim_mon_stat[0]),
		       .mon_full(eim_mon_stat[1]),
		       .mon_data(mon_data[23:0]),
		       .bclk_reset(bclk_reset)
		       );
			 
   //////////////
   //////////////
   // DDR3 interface macro
   //////////////
   //////////////

   wire c1_clk0, c1_rst0;

   /// remember: you need to edit the IBUFG in ddr3_if_4port/user_design/rtl/infrastructure.v to a BUFG!
   ddr3_clkgen ddr3_clkgen (
			    .clk50in(clk),
			    .clk50(clk50),
			    .clk400(ddr3clk),
			    .clk100(clk100),
			    .clk25(clk25),
			    .clk12p5(clk12p5),
			    .RESET(dll_reset),
			    .LOCKED(ddr3_dll_locked)
			    );

   wire p2_cmd_en_pulse, p2_wr_en_pulse, p3_cmd_en_pulse, p3_rd_en_pulse;
   rising_edge p2cmdp2e( .clk(bclk_dll), .level(ddr3_p2_cmd_en), .pulse(p2_cmd_en_pulse) );
   rising_edge p2wrp2e( .clk(bclk_dll), .level(ddr3_p2_wr_en), .pulse(p2_wr_en_pulse) );
   rising_edge p3cmdp2e( .clk(bclk_dll), .level(ddr3_p3_cmd_en), .pulse(p3_cmd_en_pulse) );
   rising_edge p2rdp2e( .clk(bclk_dll), .level(ddr3_p3_rd_en), .pulse(p3_rd_en_pulse) );

   // add mux to switch between CPU-write to DDR3 memory, and logger writing to DDR3
   wire p2_cmd_en;
   wire [2:0] p2_cmd_instr;
   wire [5:0] p2_cmd_bl;
   wire [29:0] p2_cmd_byte_addr;
   wire       p2_wr_en;
   wire [3:0] p2_wr_mask;
   wire [31:0] p2_wr_data;

   reg 	       log_cmd_en_delay;
   
   assign p2_cmd_en = p2_cmd_en_pulse;
   assign p2_cmd_instr[2:0] = ddr3_p2_cmd_instr[2:0];
   assign p2_cmd_bl = ddr3_p2_cmd_bl;
   assign p2_cmd_byte_addr = ddr3_p2_cmd_byte_addr;
   assign p2_wr_en = p2_wr_en_pulse || (ddr3_p2_wr_pulse & p2_wr_pulse_gate);
   assign p2_wr_mask = ddr3_p2_wr_mask;
   assign p2_wr_data = ddr3_p2_wr_data;

   wire p3_cmd_en;
   wire p3_burst_cmd_en;
   wire [2:0] p3_cmd_instr;
   wire [2:0] p3_burst_cmd_instr;
   wire [5:0] p3_cmd_bl;
   wire [5:0] p3_burst_cmd_bl;
   wire [29:0] p3_cmd_byte_addr;
   wire [29:0] p3_burst_addr;
   wire        p3_rd_en;
   wire        p3_burst_rd_en;

   assign p3_cmd_en = burst_mode ? p3_burst_cmd_en : p3_cmd_en_pulse;
   assign p3_cmd_instr = burst_mode ? p3_burst_cmd_instr : ddr3_p3_cmd_instr;
   assign p3_cmd_bl = burst_mode ? p3_burst_cmd_bl : ddr3_p3_cmd_bl;
   assign p3_cmd_byte_addr = burst_mode ? p3_burst_addr : ddr3_p3_cmd_byte_addr;
   assign p3_rd_en = burst_mode ? p3_burst_rd_en : (p3_rd_en_pulse || (ddr3_p3_rd_pulse & p3_rd_pulse_gate));

   ddr3_eim_burst eim_burst(.bclk(bclk_i),
			    .bus_ad(eim_din),
			    .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
			    .rbk_d(ro_d_b),
			    
			    .ddr3_rd_cmd(p3_burst_cmd_instr),
			    .ddr3_rd_bl(p3_burst_cmd_bl),
			    .ddr3_rd_adr(p3_burst_addr),
			    .ddr3_rd_cmd_en(p3_burst_cmd_en),
			    .ddr3_rd_cmd_empty(ddr3_p3_cmd_empty),
			    .ddr3_rd_cmd_full(ddr3_p3_cmd_full),

			    .ddr3_rd_data(ddr3_p3_rd_data[31:0]),
			    .ddr3_rd_count(ddr3_p3_rd_count),
			    .ddr3_rd_empty(ddr3_p3_rd_empty),
			    .ddr3_rd_full(ddr3_p3_rd_full),
			    .ddr3_rd_en(p3_burst_rd_en),

			    .clear_error(rbk_clear_error),
			    .full_error(rbk_full_error),
			    .empty_error(rbk_empty_error),

			    .enable(rbk_enable),
			    .init(rbk_init),
			    .startpage(rbk_startpage[17:0]),
			    .burstlen(rbk_burstlen[15:0]),
			    
			    .bclk_reset(bclk_reset)
			    );

   eim_burstcnt burstcnt(
			 .bclk(bclk_i),
			 .bus_ad(eim_din),
			 .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
			 .measured_burst(measured_burst)
			 );
   
   ddr3_if_4port # (
    .C1_P0_MASK_SIZE(8),
    .C1_P0_DATA_PORT_SIZE(64),
    .C1_P1_MASK_SIZE(4),
    .C1_P1_DATA_PORT_SIZE(32),
    .DEBUG_EN(0),
    .C1_MEMCLK_PERIOD(2500),
    .C1_CALIB_SOFT_IP("TRUE"),
    .C1_SIMULATION("FALSE"),
    .C1_RST_ACT_LOW(0),
    .C1_INPUT_CLK_TYPE("SINGLE_ENDED"),
    .C1_MEM_ADDR_ORDER("ROW_BANK_COLUMN"),
    .C1_NUM_DQ_PINS(16),
    .C1_MEM_ADDR_WIDTH(14),
    .C1_MEM_BANKADDR_WIDTH(3)
	      )
   u_ddr3_if (

	      .c1_sys_clk             (ddr3clk),
	      .c1_sys_rst_i           (reset),
	      
	      .mcb1_dram_dq           (F_DDR3_D[15:0]),  
	      .mcb1_dram_a            (F_DDR3_A[13:0]),  
	      .mcb1_dram_ba           (F_BA[2:0]),
	      .mcb1_dram_ras_n        (F_RAS_N),                        
	      .mcb1_dram_cas_n        (F_CAS_N),                        
	      .mcb1_dram_we_n         (F_WE_N),                          
	      .mcb1_dram_odt          (F_DDR3_ODT),
	      .mcb1_dram_cke          (F_DDR3_CKE),                          
	      .mcb1_dram_ck           (F_DDR3_CK_P),                          
	      .mcb1_dram_ck_n         (F_DDR3_CK_N),       
	      .mcb1_dram_dqs          (F_LDQS_P),                          
	      .mcb1_dram_dqs_n        (F_LDQS_N),
	      .mcb1_dram_udqs         (F_UDQS_P),    // for X16 parts   
	      .mcb1_dram_udqs_n       (F_UDQS_N),  // for X16 parts
	      .mcb1_dram_udm          (F_UDM),     // for X16 parts
	      .mcb1_dram_dm           (F_LDM),
	      .mcb1_dram_reset_n      (F_DDR3_RST_N),
	      .c1_clk0		        (c1_clk0),
	      .c1_rst0		        (c1_rst0),
	
	      .c1_calib_done    (ddr3_calib_done),
	      .mcb1_rzq               (F_DDR3_RZQ),  
	      .mcb1_zio               (F_DDR3_ZIO),

	      .c1_p0_cmd_clk                          (adc_oclk),
	      .c1_p0_cmd_en                           (adc_ddr3_cmd_en),
	      .c1_p0_cmd_instr                        (adc_ddr3_cmd_instr),
	      .c1_p0_cmd_bl                           (adc_ddr3_cmd_bl),
	      .c1_p0_cmd_byte_addr                    (adc_ddr3_cmd_byte_addr),
	      .c1_p0_cmd_empty                        (adc_ddr3_cmd_empty),
	      .c1_p0_cmd_full                         (adc_ddr3_cmd_full),
	      .c1_p0_wr_clk                           (adc_oclk),
	      .c1_p0_wr_en                            (adc_ddr3_wr_en),
	      .c1_p0_wr_mask                          (adc_ddr3_wr_mask),
	      .c1_p0_wr_data                          (adc_ddr3_wr_data),
	      .c1_p0_wr_full                          (adc_ddr3_wr_full),
	      .c1_p0_wr_empty                         (adc_ddr3_wr_empty),
	      .c1_p0_wr_count                         (adc_ddr3_wr_count),
	      .c1_p0_wr_underrun                      (adc_ddr3_wr_underrun),
	      .c1_p0_wr_error                         (adc_ddr3_wr_error),
	      .c1_p0_rd_clk                           (adc_oclk), // read function not used
	      .c1_p0_rd_en                            (1'b0),
//	      .c1_p0_rd_data                          (adc_ddr3_rd_data),   // read function is not used
//	      .c1_p0_rd_full                          (adc_ddr3_rd_full),
//	      .c1_p0_rd_empty                         (adc_ddr3_rd_empty),
//	      .c1_p0_rd_count                         (adc_ddr3_rd_count),
//	      .c1_p0_rd_overflow                      (adc_ddr3_rd_overflow),
//	      .c1_p0_rd_error                         (adc_ddr3_rd_error),
	      
	      // port 2 dedicated to CPU debug interface
	      .c1_p2_cmd_clk                          (bclk_dll),
	      .c1_p2_cmd_en                           (p2_cmd_en),
	      .c1_p2_cmd_instr                        (p2_cmd_instr),
	      .c1_p2_cmd_bl                           (p2_cmd_bl),
	      .c1_p2_cmd_byte_addr                    (p2_cmd_byte_addr),
	      .c1_p2_cmd_empty                        (ddr3_p2_cmd_empty),
	      .c1_p2_cmd_full                         (ddr3_p2_cmd_full),
	      .c1_p2_wr_clk                           (bclk_dll),
	      .c1_p2_wr_en                            (p2_wr_en),
	      .c1_p2_wr_mask                          (p2_wr_mask),
	      .c1_p2_wr_data                          (p2_wr_data),
	      .c1_p2_wr_full                          (ddr3_p2_wr_full),
	      .c1_p2_wr_empty                         (ddr3_p2_wr_empty),
	      .c1_p2_wr_count                         (ddr3_p2_wr_count),
	      .c1_p2_wr_underrun                      (ddr3_p2_wr_underrun),
	      .c1_p2_wr_error                         (ddr3_p2_wr_error),

	      // port 1 for burst read access
	      .c1_p1_cmd_clk                          (bclk_i),
	      .c1_p1_cmd_en                           (p3_cmd_en),
	      .c1_p1_cmd_instr                        (p3_cmd_instr),
	      .c1_p1_cmd_bl                           (p3_cmd_bl),
	      .c1_p1_cmd_byte_addr                    (p3_cmd_byte_addr),
	      .c1_p1_cmd_empty                        (ddr3_p3_cmd_empty),
	      .c1_p1_cmd_full                         (ddr3_p3_cmd_full),
	      .c1_p1_rd_clk                           (bclk_i),
	      .c1_p1_rd_en                            (p3_rd_en),
	      .c1_p1_rd_data                          (ddr3_p3_rd_data),
	      .c1_p1_rd_full                          (ddr3_p3_rd_full),
	      .c1_p1_rd_empty                         (ddr3_p3_rd_empty),
	      .c1_p1_rd_count                         (ddr3_p3_rd_count),
	      .c1_p1_rd_overflow                      (ddr3_p3_rd_overflow),
	      .c1_p1_rd_error                         (ddr3_p3_rd_error),
	      .c1_p1_wr_clk (bclk_i),
	      .c1_p1_wr_en (1'b0)

	      // not used
//	      .c1_p4_cmd_clk                          (bclk_dll),
//	      .c1_p4_cmd_en                           (1'b0),
//	      .c1_p4_cmd_instr                        (ddr3_wr_cmd_instr[2:0]),
//	      .c1_p4_cmd_bl                           (ddr3_wr_cmd_bl[5:0]),
//	      .c1_p4_cmd_byte_addr                    (ddr3_wr_adr[29:0]),
//	      .c1_p4_cmd_empty                        (ddr3_wr_cmd_empty),
//	      .c1_p4_cmd_full                         (ddr3_wr_cmd_full),
//	      .c1_p4_wr_clk                           (bclk_dll),
//	      .c1_p4_wr_en                            (1'b0),
//	      .c1_p4_wr_mask                          (ddr3_wr_mask[3:0]),
//	      .c1_p4_wr_data                          (ddr3_wr_dat[31:0]),
//	      .c1_p4_wr_full                          (ddr3_wr_full),
//	      .c1_p4_wr_empty                         (ddr3_wr_empty),
//	      .c1_p4_wr_underrun                      
//	      .c1_p4_wr_error                         
//	      .c1_p4_wr_count                         (ddr3_wr_dat_count[6:0]),

	      // not used
//	      .c1_p5_cmd_clk                          (bclk_dll),
//	      .c1_p5_cmd_en                           (1'b0),
//	      .c1_p5_cmd_instr                        (ddr3_rd_cmd_instr[2:0]),
//	      .c1_p5_cmd_bl                           (ddr3_rd_cmd_bl[5:0]),
//	      .c1_p5_cmd_byte_addr                    (ddr3_rd_adr[29:0]),
//	      .c1_p5_cmd_empty                        (c1_p5_cmd_empty),
//	      .c1_p5_cmd_full                         (ddr3_rd_cmd_full),
//	      .c1_p5_rd_clk                           (bclk_dll),
//	      .c1_p5_rd_en                            (1'b0),
//	      .c1_p5_rd_data                          (ddr3_rd_dat[31:0]),
//	      .c1_p5_rd_full                          (ddr3_rd_dat_full),
//	      .c1_p5_rd_empty                         (ddr3_rd_dat_empty),
//	      .c1_p5_rd_count                         (ddr3_rd_dat_count[6:0]),
//	      .c1_p5_rd_overflow                      (ddr3_rd_dat_overflow)
//	      .c1_p5_rd_error                         (c1_p5_rd_error)
	      );


   // mux between block memory and register set based on high bits
   //   assign eim_dout = (bus_addr[18:16] != 3'b000) ? ro_d : bram_dout;
   // pipeline to improve timing
   reg [15:0]		     ro_d_r;
   reg [15:0] 		     ro_d_b_r;
   reg [1:0] 		     eim_rdcs;
   reg [15:0] 		     eim_dout_pipe;
   reg [15:0] 		     eim_dout_pipe2;
   
   always @(posedge bclk_dll) begin
      ro_d_r <= ro_d;
      ro_d_b_r <= ro_d_b;
      eim_rdcs[1:0] <= EIM_CS[1:0];
      eim_dout_pipe <= (eim_rdcs[1:0] == 2'b10) ? ro_d_r : ro_d_b_r;
   end

   always @(posedge bclk_o) begin
      eim_dout_pipe2 <= eim_dout_pipe; // retime near the source to allow max time for wire delay
//      eim_dout <= eim_dout_pipe2; // no time to do anything but transit between these domains
   end;


   //////////////
   // Output pipeline registers -- explicit instantiation as their LOCs are controlled in the UCF.
   //////////////
   FDSE oddr2_eim0( .D( eim_dout_pipe2[0] ),
		     .C( bclk_o ),
		     .Q( eim_dout[0] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim1( .D( eim_dout_pipe2[1] ),
		     .C( bclk_o ),
		     .Q( eim_dout[1] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim2( .D( eim_dout_pipe2[2] ),
		     .C( bclk_o ),
		     .Q( eim_dout[2] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim3( .D( eim_dout_pipe2[3] ),
		     .C( bclk_o ),
		     .Q( eim_dout[3] ),
		     .CE( 1'b1 ), .S(1'b0) );

   FDSE oddr2_eim4( .D( eim_dout_pipe2[4] ),
		     .C( bclk_o ),
		     .Q( eim_dout[4] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim5( .D( eim_dout_pipe2[5] ),
		     .C( bclk_o ),
		     .Q( eim_dout[5] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim6( .D( eim_dout_pipe2[6] ),
		     .C( bclk_o ),
		     .Q( eim_dout[6] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim7( .D( eim_dout_pipe2[7] ),
		     .C( bclk_o ),
		     .Q( eim_dout[7] ),
		     .CE( 1'b1 ), .S(1'b0) );

   FDSE oddr2_eim8( .D( eim_dout_pipe2[8] ),
		     .C( bclk_o ),
		     .Q( eim_dout[8] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim9( .D( eim_dout_pipe2[9] ),
		     .C( bclk_o ),
		     .Q( eim_dout[9] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eimA( .D( eim_dout_pipe2[10] ),
		     .C( bclk_o ),
		     .Q( eim_dout[10] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eimB( .D( eim_dout_pipe2[11] ),
		     .C( bclk_o ),
		     .Q( eim_dout[11] ),
		     .CE( 1'b1 ), .S(1'b0) );

   FDSE oddr2_eimC( .D( eim_dout_pipe2[12] ),
		     .C( bclk_o ),
		     .Q( eim_dout[12] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eimD( .D( eim_dout_pipe2[13] ),
		     .C( bclk_o ),
		     .Q( eim_dout[13] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eimE( .D( eim_dout_pipe2[14] ),
		     .C( bclk_o ),
		     .Q( eim_dout[14] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eimF( .D( eim_dout_pipe2[15] ),
		     .C( bclk_o ),
		     .Q( eim_dout[15] ),
		     .CE( 1'b1 ), .S(1'b0) );
   
   //////////////
   // IOBUFs as required by design
   //////////////
   IBUFGDS clkibufgds( .I(CLK2_P), .IB(CLK2_N), .O(clk) );

   
   reg [15:0]		      eim_d_t;
//   assign eim_d_t = EIM_OE | !EIM_LBA;
   reg 		      eim_lba_reg;
   reg 		      eim_oe_reg;
   
   always @(posedge bclk_i) begin
      eim_lba_reg <= EIM_LBA;
      eim_oe_reg <= EIM_OE;
   end
   
   always @(posedge bclk_o) begin
      eim_d_t[ 0] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 1] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 2] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 3] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 4] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 5] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 6] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 7] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 8] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 9] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[10] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[11] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[12] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[13] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[14] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[15] = eim_oe_reg | !eim_lba_reg;
   end
   
   
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim0 (.IO(EIM_DA[ 0]), .I(eim_dout[ 0]), .T(eim_d_t[0]), .O(eim_din[ 0]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim1 (.IO(EIM_DA[ 1]), .I(eim_dout[ 1]), .T(eim_d_t[1]), .O(eim_din[ 1]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim2 (.IO(EIM_DA[ 2]), .I(eim_dout[ 2]), .T(eim_d_t[2]), .O(eim_din[ 2]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim3 (.IO(EIM_DA[ 3]), .I(eim_dout[ 3]), .T(eim_d_t[3]), .O(eim_din[ 3]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim4 (.IO(EIM_DA[ 4]), .I(eim_dout[ 4]), .T(eim_d_t[4]), .O(eim_din[ 4]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim5 (.IO(EIM_DA[ 5]), .I(eim_dout[ 5]), .T(eim_d_t[5]), .O(eim_din[ 5]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim6 (.IO(EIM_DA[ 6]), .I(eim_dout[ 6]), .T(eim_d_t[6]), .O(eim_din[ 6]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim7 (.IO(EIM_DA[ 7]), .I(eim_dout[ 7]), .T(eim_d_t[7]), .O(eim_din[ 7]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim8 (.IO(EIM_DA[ 8]), .I(eim_dout[ 8]), .T(eim_d_t[8]), .O(eim_din[ 8]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim9 (.IO(EIM_DA[ 9]), .I(eim_dout[ 9]), .T(eim_d_t[9]), .O(eim_din[ 9]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim10 (.IO(EIM_DA[10]), .I(eim_dout[10]), .T(eim_d_t[10]), .O(eim_din[10]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim11 (.IO(EIM_DA[11]), .I(eim_dout[11]), .T(eim_d_t[11]), .O(eim_din[11]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim12 (.IO(EIM_DA[12]), .I(eim_dout[12]), .T(eim_d_t[12]), .O(eim_din[12]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim13 (.IO(EIM_DA[13]), .I(eim_dout[13]), .T(eim_d_t[13]), .O(eim_din[13]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim14 (.IO(EIM_DA[14]), .I(eim_dout[14]), .T(eim_d_t[14]), .O(eim_din[14]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim15 (.IO(EIM_DA[15]), .I(eim_dout[15]), .T(eim_d_t[15]), .O(eim_din[15]));

   //////////////
   /// "heartbeat" counter
   //////////////
   always @(posedge clk50) begin
      counter <= counter + 1;
   end

   assign FPGA_LED2 = counter[23];

   //////////////
   // tie downs (unused signals as of this rev of design)
   //////////////
   assign APOPTOSIS = 1'b0; // make apoptosis inactive, tigh high to force reboot on config
   assign ECSPI3_MISO = 1'b0;
   
endmodule
