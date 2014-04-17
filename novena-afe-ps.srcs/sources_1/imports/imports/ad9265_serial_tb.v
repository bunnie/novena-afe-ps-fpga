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

module ad9265_serial_tb;
   reg clk;
   reg [7:0] data;
   reg 	     rd_wr_n;
   reg [12:0]  addr;
   reg 	      commit;
   wire       busy;
   reg 	      data_in;
   
   parameter PERIOD = 16'd80;   // 12.5 MHz
   always begin
      clk = 1'b0;
      #(PERIOD/2) clk = 1'b1;
      #(PERIOD/2);
   end

   wire ADC_SCLK, ADC_SDATA, ADC_SCS, ADC_DRIVE;
   wire [7:0] rdata;
   
   ad9265_serial adcserial (
			     .sclk(ADC_SCLK),
			     .sdata(ADC_SDATA),
			     .scs(ADC_SCS),
			     .sdata_in(data_in),
			     .sdata_drv(ADC_DRIVE),
			     .w_data(data),
			     .r_data(rdata),
			     .rd_wr_n(rd_wr_n),
			     .addr(addr),
			     .commit(commit),
			     .busy(busy),
			     .clk12p5(clk)
			     );

   initial begin
      data = 8'b0;
      addr = 13'b0;
      commit = 1'b0;
      rd_wr_n = 0;
      data_in = 0;
            
      $stop;

      // reset at gate level
      #(PERIOD*16);

      data = 8'h1F;
      #(PERIOD*16);
      
      addr = 13'h3;
      commit = 1'b1;
      #(PERIOD*10);
      commit = 1'b0;

      data = 8'h96;
      #(PERIOD*100);
      
      addr = 13'h81;
      commit = 1'b1;
      #(PERIOD*10);

      commit = 1'b0;
      #(PERIOD*100);

      rd_wr_n = 1;
      addr = 13'h95a;
      #(PERIOD*10);
      commit = 1'b1;
      #(PERIOD*20);

      data_in = 1;
      #(PERIOD);
      data_in = 0;
      #(PERIOD);
      data_in = 1;
      #(PERIOD);
      data_in = 0;
      #(PERIOD);

      data_in = 0;
      #(PERIOD);
      data_in = 0;
      #(PERIOD);
      data_in = 0;
      #(PERIOD);
      data_in = 1;
      #(PERIOD);

      data_in = 0;

      #(PERIOD*100);
      $stop;
   end // initial begin
endmodule // tadc08d1020_serial_tb


