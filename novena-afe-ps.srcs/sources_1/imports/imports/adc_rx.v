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

module adc_rx(
	      input wire [7:0] data_p,
	      input wire [7:0] data_n,

	      input wire clk_p,
	      input wire clk_n,

	      output reg [63:0] adc_dat,
	      output wire oclk,
	      output wire oclkx2,
	      
	      input wire reset
	      );

   wire 	      adc_gclk;
   wire 	      adc_bitslip;
   wire 	      adc_pll_locked;
   wire 	      adc_bufpll_locked;
   wire 	      adc_clk;

   wire [63:0] 	      adc_fast;

   always @(posedge oclk) begin
      // swaps:   1011_1101  // swaps due to P/N pair swapping for routability
      adc_dat <= adc_fast ^ 64'hBDBD_BDBD_BDBD_BDBD;
   end

   serdes_1_to_n_clk_pll_s8_diff input_adc_clk (
						.clkin_p(clk_p), 
						.clkin_n(clk_n), 
						.rxioclk(adc_clk), 
						.pattern1(2'b10), 
						.pattern2(2'b01), 
						.rx_serdesstrobe(adc_serdesstrobe), 
						.reset(reset), 
						.rx_bufg_pll_x1(adc_gclk), 
						.rx_pll_lckd(adc_pll_locked),
						.rx_pllout_div8(oclk),
						.rx_pllout_div4(oclkx2),
//						.rx_pllout_xs(), 
						.bitslip(adc_bitslip), 
						.rx_bufpll_lckd(adc_bufpll_locked) 
//						.datain()
						) ;
   
   serdes_1_to_n_data_s8_diff input_adc (
					     .use_phase_detector(1'b1), 
					     .datain_p(data_p[7:0]), 
					     .datain_n(data_n[7:0]), 
					     .rxioclk(adc_clk), 
					     .rxserdesstrobe(adc_serdesstrobe), 
					     .reset(reset), 
					     .gclk(oclk), 
					     .bitslip(adc_bitslip),
//					     .debug_in(), 
//					     .debug(),
					     .data_out(adc_fast) 
					     ) ;

endmodule // adc_rx
