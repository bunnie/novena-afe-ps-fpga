module ad9265_serial(
		     output wire sclk,  // limit 25 MHz, running at 12.5MHz
		     output reg sdata,
		     output reg scs,
		     input wire sdata_in,
		     output reg sdata_drv,

		     input wire [7:0] w_data,
		     output reg [7:0] r_data,
		     input wire [12:0] addr,
		     input rd_wr_n,  // write if low, read if high
		     input wire commit,
		     output wire busy,
		     input wire clk12p5
		     );
   

   reg [7:0] 			    l_data;
   reg [12:0] 			    l_addr;
   reg 				    l_commit;
   reg 				    l_busy;
   reg 				    l_rd_wr_n;

   reg [5:0] 			    cycle;
   reg [23:0] 			    shifter;
   reg 				    commit_d;
   reg 				    commit_pulse;  // get the rising edge only of commit

   reg [7:0] 			    shift_in;

   assign busy = l_busy;
   
   // register i2c bus signals into local clock domain to ease timing
   always @(posedge clk12p5) begin
      l_data <= w_data;
      l_addr <= addr;
      l_commit <= commit;
      l_rd_wr_n <= rd_wr_n;

      // busy whenever the cycle counter isn't at 0
      l_busy <= (cycle[5:0] != 6'b0);
   end

   // turn commit into a locally timed pulse
   always @(posedge clk12p5) begin
      commit_d <= l_commit;
      commit_pulse <= !commit_d && l_commit;
   end

   // forward the clock net
   ODDR2 sclk_oddr2 (
		     .D0(1'b1),
		     .D1(1'b0),
		     .C0(clk12p5),
		     .C1(!clk12p5),
		     .CE(1'b1),
		     .R(1'b0),
		     .S(1'b0),
		     .Q(sclk) );

   reg readover;
   // main shifter logic
   always @(posedge clk12p5) begin
      if( commit_pulse && (cycle[5:0] == 6'b0) ) begin
	 shift_in <= shift_in;
	 sdata_drv <= 1;
	 shifter[23:0] <= {l_rd_wr_n,2'b00,l_addr[12:0],l_data[7:0]};
	 cycle[5:0] <= 6'b01_1000;
	 readover <= 1'b1;
      end else if( (cycle[5:0] != 6'b0) || ((cycle[5:0] == 6'b0) && readover && l_rd_wr_n) ) begin
	 if( l_rd_wr_n == 1'b0 ) begin // write
	    sdata_drv <= 1;
	    cycle[5:0] <= cycle[5:0] - 6'b1;
	    shifter[23:0] <= {shifter[22:0], 1'b0};
	    shift_in <= shift_in;
	    readover <= 1'b1;
	 end else begin // read
	    shifter[23:0] <= {shifter[22:0], 1'b0};
	    if( cycle[5:0] < 6'h9 ) begin
	       sdata_drv <= 0;
	    end else begin
	       sdata_drv <= 1;
	    end
	      
	    if( cycle[5:0] < 6'h8 ) begin
	       shift_in[7:0] <= {shift_in[6:0],sdata_in};
	    end else begin
	       shift_in <= shift_in;
	    end
	    
	    if( cycle[5:0] == 6'b0 ) begin
	       cycle[5:0] <= 6'b0;
	       readover <= 1'b0;
	    end else begin
	       cycle[5:0] <= cycle[5:0] - 6'b1;
	       readover <= readover;
	    end
	 end
      end else begin // if ( cycle[5:0] != 6'b0 )
	 readover <= 1'b0;
	 shift_in <= shift_in;
	 sdata_drv <= 1;
	 cycle[5:0] <= 6'b0;
	 shifter[23:0] <= 24'b0;
      end
   end

   // output stage logic
   always @(posedge clk12p5) begin
      sdata <= shifter[23];
      scs <= !(cycle[5:0] != 6'b0);

      r_data <= shift_in; // add a register stage to ease timing constraints
   end

endmodule // ad9265_serial

