/////////////////////////////////
// internal signals            //
/////////////////////////////////

// SHUTTER outputs
reg SHUTTER;
wire INVERTED_SHUTTER;
wire [2:0]SEL_SHUTTER;

// OPIOM outputs -> MUSST inputs
reg ITRIG;
wire [1:0]SEL_ITRIG;
reg STEP_OUT;
// MUSST outputs -> OPIOM inputs
wire ATRIG;
wire BTRIG;

//TRIG_IN
wire TRIG_IN,MUSST_TRIG;
//COUNTER
wire COUNTERS_IN;
reg COUNTERS_OUT;

// MCA DETECTORS;
wire TRIG_OUT_MCA1;
wire SEL_MCA1;

// CAMERA DETECTORS
wire TRIG_OUT_CAM1,TRIG_OUT_CAM2,TRIG_OUT_CAM3,TRIG_OUT_CAM4;
wire SEL_CAM1,SEL_CAM2,SEL_CAM3,SEL_CAM4;
wire SOR_CAM1,SOR_CAM2;
wire SHUTTER_CAM1,SHUTTER_CAM2;

//MOTORS
wire [1:0]SEL_MOT_OUT;
wire STEP_MOT1,STEP_MOT3,STEP_MOT4;
reg BOOST_OUT;
wire BOOST_MOT1,BOOST_MOT2,BOOST_MOT3,BOOST_MOT4;

//Clock

reg   [7:0]CLK_64K;
reg   [2:0]TIMER_PHOTONIC;
wire  CLK_62K5;
reg  TRIG_PHOTONIC;
/////////////////////////////////

assign ATRIG   			= I1;   		// Input from Musst ATRIG.
assign BTRIG     		= I2;   		// Input from Musst BTRIG
assign SOR_CAM1 		= I3;   		// SOR camera 1
assign SHUTTER_CAM1 		= I4;   		// shutter cam 1
assign SOR_CAM2 		= I5;			// SOR camera 2
assign SHUTTER_CAM2 		= I6;	  		// shutter camera 2
assign BOOST_MOT1  		= I7;   		// Boost/Move motor 1
assign BOOST_MOT2 		= I8;   		// Boost/Move from motor2.

assign STEP_MOT1     		= IB1;   		// Step motor 1
assign BOOST_MOT3 		= IB2;  		// Boost from motor3.
assign STEP_MOT3 		= IB3;  		// step from motor3.
assign BOOST_MOT4 		= IB4;  		// Boost from motor4.
assign STEP_MOT4 		= IB5;  		// Vstep from motor4.
assign COUNTERS_IN    		= IB6;   		// Input from COUNTER card.
assign SOR_CAM3 		= IB7;  		// SOR camera 3
assign SHUTTER_CAM3 		= IB8;  		// shutter camera 3

assign O1 			= SHUTTER;            	// SHUTTER Output
assign O2 			= COUNTERS_OUT;         // VCT6/P201.
assign O3 			= TRIG_OUT_MCA1;        // MCA 1 like XIA
assign O5 			= TRIG_OUT_CAM1;	// cam 1
assign O6 			= TRIG_OUT_CAM2;	// cam 2
assign O7 			= TRIG_OUT_CAM3;	// cam 3
assign O8 			= TRIG_OUT_CAM4;	// cam 4

//assign OB1 = ITRIG;
assign O4 = ITRIG;
assign OB2 = STEP_OUT;
assign OB3 = BOOST_OUT;

assign SEL_SHUTTER[2:0] 	= {IM3,IM2,IM1}; 	// Inp. mult. sel. for shutter source.
assign SEL_ITRIG[1:0]  		= {IM5,IM4};		// ITRIG Musst
assign SEL_MOT_OUT[1:0]   	= {IM7,IM6};    	// Inp. mult. sel. for step motor

assign SEL_MCA1 		= IMA1;

assign SEL_CAM1 		= IMA3;
assign SEL_CAM2 		= IMA4;
assign SEL_CAM3 		= IMA5;
assign SEL_CAM4 		= IMA6;

assign INVERTED_SHUTTER 	= IMA7;
assign DETECTOR_TRIG_MUSST 	= IMA8;
assign SEL_MUSST_TRIG		= IM8;
wire SEL_MUSST_INVERTED;
assign SEL_MUSST_INVERTED       = IMA2;
////////////////////////////////////////////////////////////////////////////////
//		       source for output ITRIG                                //
////////////////////////////////////////////////////////////////////////////////
always @(SEL_ITRIG or SOR_CAM1 or SOR_CAM2 or SOR_CAM3)
  begin
     case (SEL_ITRIG)
       2'b01 : ITRIG = SOR_CAM1;
       2'b10 : ITRIG = SOR_CAM2;
       2'b11 : ITRIG = SOR_CAM3;
       default : ITRIG = 1'b0;
     endcase
  end
   
always @(SEL_MOT_OUT or
	 BOOST_MOT1 or BOOST_MOT2 or BOOST_MOT3 or BOOST_MOT4 or 
	 STEP_MOT1 or STEP_MOT3 or STEP_MOT4)
  begin
     case (SEL_MOT_OUT)
       2'b01 :
	 begin
	    STEP_OUT = 1'b0;
	    BOOST_OUT = BOOST_MOT2;
	 end
       2'b10 : 
	 begin
	    STEP_OUT = STEP_MOT3;
	    BOOST_OUT = BOOST_MOT3;
	 end
       2'b11 : 
	 begin
	    STEP_OUT = STEP_MOT4;
	    BOOST_OUT = BOOST_MOT4;
	 end
       default : 
	 begin
	    STEP_OUT = STEP_MOT1;
	    BOOST_OUT = BOOST_MOT1;
	 end
     endcase
  end // always @ (SEL_MOT_OUT or...
   ////////

/////////////////////////////////
// shutter input multiplexers  //
/////////////////////////////////
   
always @(SEL_SHUTTER or INVERTED_SHUTTER or BTRIG or 
	 SHUTTER_CAM1 or SHUTTER_CAM2 or SHUTTER_CAM3 or
	 COUNTERS_IN)
  begin
     case (SEL_SHUTTER)
       3'b001 : SHUTTER = ~INVERTED_SHUTTER;
       3'b010 : SHUTTER = INVERTED_SHUTTER ^ BTRIG;
       3'b011 : SHUTTER = INVERTED_SHUTTER ^ SHUTTER_CAM1;
       3'b100 : SHUTTER = INVERTED_SHUTTER ^ SHUTTER_CAM2;
       3'b101 : SHUTTER = INVERTED_SHUTTER ^ SHUTTER_CAM3;
       3'b110 : SHUTTER = INVERTED_SHUTTER ^ COUNTERS_IN;
       default : SHUTTER = INVERTED_SHUTTER;
     endcase
  end // always @ (SEL_SHUTTER or INVERTED_SHUTTER or BTRIG or...

////////////////////////////////////////////////////////////////////////////////
//			    trig selection                                    //
////////////////////////////////////////////////////////////////////////////////
assign MUSST_TRIG = SEL_MUSST_TRIG ? BTRIG : ATRIG;
assign TRIG_IN = DETECTOR_TRIG_MUSST ? (MUSST_TRIG ^ SEL_MUSST_INVERTED) : COUNTERS_IN;

////////////////////////////////////////////////////////////////////////////////
//				 MCA                                          //
////////////////////////////////////////////////////////////////////////////////
assign TRIG_OUT_MCA1 = SEL_MCA1 ? TRIG_IN : 1'b0;

////////////////////////////////////////////////////////////////////////////////
//				 CAMERA                                       //
////////////////////////////////////////////////////////////////////////////////
assign TRIG_OUT_CAM1 = SEL_CAM1 ? TRIG_IN : 1'b0;
assign TRIG_OUT_CAM2 = SEL_CAM2 ? TRIG_IN : 1'b0;
assign TRIG_OUT_CAM3 = SEL_CAM3 ? TRIG_PHOTONIC : 1'b0;
assign TRIG_OUT_CAM4 = SEL_CAM4 ? TRIG_PHOTONIC : 1'b0;
   
always @(posedge CLK16)
   CLK_64K <= CLK_64K + 1;


assign CLK_62K5 = CLK_64K[7];

always @(posedge TRIG_IN or posedge CLK_62K5)
  begin
     if(TRIG_IN)
       TIMER_PHOTONIC <= 3'b111;
     else
       TIMER_PHOTONIC<= TIMER_PHOTONIC - 1;
  end

always @(posedge TRIG_IN or negedge CLK_62K5)
  begin
     if(TRIG_IN)
       TRIG_PHOTONIC = 1'b1;
     eles if(TIMER_PHOTONIC == 3'b000)
       TRIG_PHOTONIC = 1'b0;
  end
