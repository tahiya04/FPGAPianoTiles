

module RhythmRush(	
			CLOCK_50,						
			SW, 								
			HEX0,								
			HEX2,	
			HEX4,
			KEY,							   
			VGA_CLK,   						
			VGA_HS,							
			VGA_VS,							
			VGA_BLANK_N,					
			VGA_SYNC_N,						
			VGA_R,   						
			VGA_G,	 						
			VGA_B,   						   
			
			// audio ports

			AUD_ADCDAT,

			// Bidirectionals
			AUD_BCLK,
			AUD_ADCLRCK,
			AUD_DACLRCK,

			FPGA_I2C_SDAT,

			// Outputs
			AUD_XCK,
			AUD_DACDAT,

			FPGA_I2C_SCLK
		);
		
		
	input		    CLOCK_50;			
	input	 [3:0] KEY;					
	input  [9:0] SW;					
	output [6:0] HEX0;				
	output [6:0] HEX2;
	output [6:0] HEX4;
	output		 VGA_CLK;   		
	output		 VGA_HS;				
	output		 VGA_VS;				
	output		 VGA_BLANK_N;		
	output		 VGA_SYNC_N;		
	output [7:0] VGA_R;   			
	output [7:0] VGA_G;	 			
	output [7:0] VGA_B;   			
	
	
		//// AUDIO FROM: EECG WEBSITE, REFERENCE CODE ALTERED AND INSTANTIATED
		/*****************************************************************************
	 *                           Parameter Declarations                          *
	 *****************************************************************************/


	/*****************************************************************************
	 *                             Port Declarations                             *
	 *****************************************************************************/
	// Inputs
	//input				CLOCK_50;
	//input		[3:0]	KEY;
	//input		[3:0]	SW;

	input				AUD_ADCDAT;

	// Bidirectionals
	inout				AUD_BCLK;
	inout				AUD_ADCLRCK;
	inout				AUD_DACLRCK;

	inout				FPGA_I2C_SDAT;

	// Outputs
	output				AUD_XCK;
	output				AUD_DACDAT;

	output				FPGA_I2C_SCLK;

	/*****************************************************************************
	 *                 Internal Wires and Registers Declarations                 *
	 *****************************************************************************/
	// Internal Wires
	wire				audio_in_available;
	wire		[31:0]	left_channel_audio_in;
	wire		[31:0]	right_channel_audio_in;
	wire				read_audio_in;

	wire				audio_out_allowed;
	wire		[31:0]	left_channel_audio_out;
	wire		[31:0]	right_channel_audio_out;
	wire				write_audio_out;

	// Internal Registers

	reg [18:0] delay_cnt;
	wire [18:0] delay;

	reg snd;

	// State Machine Registers

	/*****************************************************************************
	 *                         Finite State Machine(s)                           *
	 *****************************************************************************/


	/*****************************************************************************
	 *                             Sequential Logic                              *
	 *****************************************************************************/

	always @(posedge CLOCK_50)
		if(delay_cnt == delay) begin
			delay_cnt <= 0;
			snd <= !snd;
		end else delay_cnt <= delay_cnt + 1;

	/*****************************************************************************
	 *                            Combinational Logic                            *
	 *****************************************************************************/

	//assign delay = {4'b0001, 15'd3000};
	// try this
	assign delay = {1'b0, KEY[3:1], 15'd8000};

	wire [31:0] sound = (KEY[3:1] == 3'b111) ? 0 : snd ? 32'd10000000 : -32'd10000000;


	assign read_audio_in			= audio_in_available & audio_out_allowed;

	assign left_channel_audio_out	= left_channel_audio_in+sound;
	assign right_channel_audio_out	= right_channel_audio_in+sound;
	assign write_audio_out			= audio_in_available & audio_out_allowed;

	/*****************************************************************************
	 *                              Internal Modules                             *
	 *****************************************************************************/

	Audio_Controller Audio_Controller (
		// Inputs
		.CLOCK_50						(CLOCK_50),
		.reset						(~KEY[0]),

		.clear_audio_in_memory		(),
		.read_audio_in				(read_audio_in),
		
		.clear_audio_out_memory		(),
		.left_channel_audio_out		(left_channel_audio_out),
		.right_channel_audio_out	(right_channel_audio_out),
		.write_audio_out			(write_audio_out),

		.AUD_ADCDAT					(AUD_ADCDAT),

		// Bidirectionals
		.AUD_BCLK					(AUD_BCLK),
		.AUD_ADCLRCK				(AUD_ADCLRCK),
		.AUD_DACLRCK				(AUD_DACLRCK),


		// Outputs
		.audio_in_available			(audio_in_available),
		.left_channel_audio_in		(left_channel_audio_in),
		.right_channel_audio_in		(right_channel_audio_in),

		.audio_out_allowed			(audio_out_allowed),

		.AUD_XCK					(AUD_XCK),
		.AUD_DACDAT					(AUD_DACDAT)

	);

	avconf #(.USE_MIC_INPUT(1)) avc (
		.FPGA_I2C_SCLK					(FPGA_I2C_SCLK),
		.FPGA_I2C_SDAT					(FPGA_I2C_SDAT),
		.CLOCK_50					(CLOCK_50),
		.reset						(~KEY[0])
	);
	 
		
		
	
	
	//// END AUIDO
	
	wire resetn;
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	// VGA STUFF
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	
	
	/// ABOVE STUFF IS ALL VGA STUFF
	
	wire draw;
	wire erase;
	wire DrawEnable;
	wire update;
	wire reset;
	assign resetn = KEY[0]; // key 0 is the reset
	wire assignColour = 3'b101;
	
	
	wire[12:0]frameCounter; 
	wire [7:0] xCounter;
	wire [7:0] yCounter;
	wire[25:0] EN; // essentially the rate divider
	
	////////////////////////////////////////////////////////////////
	// update with rate divider
	wire ENABLE;
	RateDivider rd(CLOCK_50, resetn, 26'd12499999, ENABLE);
	
	////////////////////////////////////////////////////////////////
	// score with onboard storage model
	wire [3:0] score;
	wire scoreEnable;
	assign scoreEnable = SW[1];
	wire [3:0]outputScore;
	
	//wire [3:0]posValue;
	wire [3:0]posValue;
	
	wire [3:0] curr_state;
	
	wire [2:0] keyType;
	
	////////////////////////////////////////////////////////////////
	/// assigning Keys
	wire KeyA;
	assign KeyA = KEY[1];
	wire KeyB;
	assign KeyB = KEY[2];
	wire KeyC;
	assign KeyC = KEY[3];
	
	
	part1 mem (
	.address(5'b00001),
	.clock(CLOCK_50),
	.data(score),
	.wren(scoreEnable),
	.q(outputScore)
   
  );
	
	
	// VGA ADAPTER
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(draw),
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "red.mif";
		
		// instantiate main control
		maincontrol u3 (
			CLOCK_50,
			resetn,
			!KEY[2],
			!KEY[1],
			!KEY[3],
			curr_state,
			keyType
			//score, 
			//posValue,
			//posValue
		);
		
		// instantiate drwaing control
		drawControl u1 (
			CLOCK_50,
			resetn,
			frameCounter,
			xCounter, 
			yCounter, 
			EN, 
			draw, 
			erase,
			update,
			drawEnable,
			reset,
			ENABLE
		);
		
		// instantiate datapath
		drawDatapath u2(
			CLOCK_50,
			resetn,
			drawEnable,
			draw, 
			erase,
			update,
			reset,
			assignColour, 
			x,
			y, 
			colour, 
			frameCounter,
			xCounter, 
			yCounter,
			EN,
			curr_state,
			score
			//scoreEnable
		);
		
		
		// hex decoder output
		hex_decoder u4(outputScore[3:0], HEX0);
		hex_decoder u5(score[3:0], HEX2);
		hex_decoder u6(curr_state[3:0], HEX4);
		
		
		
		
endmodule 

module maincontrol(
	input Clock, 
	input Resetn,
	input keyA,
	input keyB,
	input keyC,
	//output reg [3:0]score
	//output reg [3:0]posValue,
	output reg [3:0] curr_state,
	output reg [2:0] keyType
	

);

	reg [3:0] next_state;
	//reg [3:0] keyType;
	
	
	//ates 

	localparam START = 3'd0,
				  KEY1_PRESSED = 3'd1,
				  KEY2_PRESSED = 3'd2,
				  KEY3_PRESSED = 3'd3,
				  KEY1_WAIT = 3'd4,
				  KEY2_WAIT = 3'd5,
				  KEY3_WAIT  = 3'd6,
				  RESET = 3'd7,
				  FINISHED = 3'd8;
	initial
	begin
		//posValue <= 0;
		//score <= 0;
		curr_state <= START;
	end
				  
		always @(posedge Clock)
		begin
		
		/*
			case(curr_state)
				  START: if (keyA) next_state = KEY1_PRESSED;
				  KEY1_PRESSED:
					begin
						if (keyB)
							next_state = KEY1_WAIT;
						
					end
				  KEY2_PRESSED:
					begin
						if (keyC)
							next_state = KEY2_WAIT;		
					end
				  KEY3_PRESSED:
					begin
						if (keyA)
							next_state = KEY3_WAIT;
					end
				  KEY1_WAIT: 
					begin
						if (!keyA)
							next_state = KEY2_PRESSED;
					end
				  KEY2_WAIT:
				  begin
						if (!keyB)
							next_state = KEY3_PRESSED;

				  end
				  KEY3_WAIT: 
				  begin
					if (!keyC)
						next_state = FINISHED;
				  end

				  RESET: next_state = START;
				  
				  FINISHED: next_state = START;
				  
				  default: next_state = RESET;
				
			
			endcase
			
		*/
		
		case(curr_state)
				  START: next_state = KEY1_PRESSED;
				  KEY1_PRESSED:
					begin
						if (keyA)
							next_state = KEY1_WAIT;
						
					end
				  KEY2_PRESSED:
					begin
						if (keyB)
							next_state = KEY2_WAIT;		
					end
				  KEY3_PRESSED:
					begin
						if (keyC)
							next_state = KEY3_WAIT;
					end
				  KEY1_WAIT: 
					begin
						if (!keyA)
							begin
							next_state = KEY2_PRESSED;
							keyType <= 2'b00;
							end
					end
				  KEY2_WAIT:
				  begin
						if (!keyB)
							begin
							next_state = KEY3_PRESSED;
							keyType <= 2'b01;
							end

				  end
				  KEY3_WAIT: 
				  begin
					if (!keyC)
						begin
						next_state = FINISHED;
						keyType <= 2'b10;
						end
						
				  end

				  RESET: next_state = START;
				  
				  FINISHED: next_state = START;
				  
				  default: next_state = RESET;
				endcase
		
		
		end
		
		
		
		always @(posedge Clock)
			begin
				if (!Resetn)
					curr_state <= START;
				else
					curr_state <= next_state;
			end


endmodule


module drawControl(
	input Clock,
	input Resetn,
	input [12:0] frameCounter,
	input [7:0] xCount,
	input [6:0] yCount,
	input [25:0] EN,
	output reg draw,
	output reg erase,
	output reg update,
	output reg drawEnable, 
	output reg reset,
	output reg ENABLE
	
);

	reg [3:0] current_state, next_state;
	
	
	// States for the drawing
	localparam RESET   = 3'd0,
				  DRAW    = 3'd1,
				  DELAY   = 3'd2,
				  ERASE   = 3'd3,
				  UPDATE  = 3'd4;
				  
	// making state table
	always @(*)
	begin
		case(current_state)
			RESET: next_state = DRAW;
			
			DRAW: 
				if (frameCounter <= 8'd255)
					next_state = DRAW;
				else
					next_state = DELAY;
				
			DELAY:
				if (EN < 26'd5499999)
					next_state = DELAY;
				else
					next_state = ERASE;
				// if (EN < 26'd8499999)
				
			
			ERASE:
				if (frameCounter <= 8'd255) 
					next_state = ERASE;
				else 
					next_state = UPDATE;
			
			UPDATE: next_state = DRAW;
			
			default: next_state = RESET;
			
		endcase
	end
	
	
		

	// What does each state do
	
	always @(*)
	begin
		// initializing all output regs to 0
		
		draw = 1'b0;
		update = 1'b0;
		reset = 1'b0;
		erase = 1'b0;
		drawEnable = 1'b0;
		
		case(current_state)
			RESET: reset = 1'b1;
			
			DRAW: 
				begin
				draw = 1'b1;
				erase = 1'b0;
				drawEnable = 1'b1;
				end
			
			ERASE:
				begin
				draw = 1'b1; // want to draw black
				erase = 1'b1;
				drawEnable = 1'b1;
				end
			
			UPDATE:
				update = 1'b1;
				// update prev positions to current
		
		
		endcase
	
	end
	
	
	// making sure we go to the next state
	
	always @(posedge Clock)
	begin
		if (!Resetn)
			current_state <= DRAW;
		else
			current_state <= next_state;
	
	end

endmodule

module drawDatapath(
	input Clock,
	input Resetn,
	input drawEnable,
	input draw,
	input erase,
	input update,
	input reset,
	input [2:0] colour, // connected to switches
	output reg [7:0] x,
	output reg [6:0] y,
	output reg [2:0] ColourO,
	output reg [12:0] frameCounter,
	output reg [7:0] xCount,
	output reg [6:0] yCount, 
	output reg [25:0] EN,
	input [3:0] curr_state,
	
	/// score stuff
	output reg [3:0] score,
	input [2:0] keyState
	//output reg scoreEnable
	

);

	reg [7:0] tempX;
	reg [6:0] tempY;
	
	initial 
	begin
		tempX <= 8'd20;
		tempY <= 7'd0;
	end
	
	reg [1:0]posCounter;
	
	localparam BLACK = 3'b000,
				  PINK  = 3'b101;
	
	always @(posedge Clock)
	begin
	
		if (reset || ~Resetn) 
			begin
				// if reset clicked or the reset condiiton met
				tempX <= 8'd20;
				tempY <= 7'd0;
				x <= 8'd20;
				y <= 7'd0;
				frameCounter <= 11'b0;
				xCount <= 8'b0;
				yCount <= 7'b0;
				ColourO <= BLACK; // black
				EN <= 25'd0;
				
				posCounter <= 2'b0;
				
				score <= 4'b0;
				//scoreEnable <= 1'b0;
				
			
			end
		
		else
			begin
			/////////////////////////////////////////////////////////
				
				if (!erase)
					ColourO <= PINK;
					
			/////////////////////////////////////////////////////////
			
				// RATE DIVIDER !
				if (EN == 26'd5499999)
					EN <= 26'd0;
				else
					EN <= EN + 1;
					
			/////////////////////////////////////////////////////////
			
				if (drawEnable)
					begin
						if (erase)
							ColourO <= BLACK;
						else 
							ColourO <= PINK;
						
						if (frameCounter == 11'b100000000)
							frameCounter <= 11'b0;
						else
							frameCounter <= frameCounter + 1;
							
						x <= tempX + frameCounter[3:0]; 
						y <= tempY + frameCounter[7:4];
					
					end
			/////////////////////////////////////////////////////////
					
				if (update) 
					// if statements for the button being clicked 
					begin
						//ColourO <= 3'b001;
						if (y == 7'd119)
							begin
								if (posCounter == 2'b10) // MAIN CHANGE
									begin
									posCounter <= 2'b00;
									//scoreEnable <= 1'b1;
									if (curr_state == 3'd3)
										score <= score + 1;
									end
								else 
									begin
									posCounter <= posCounter + 1;
									//scoreEnable <= 1'b1;
									if (posCounter == 2'b00 && curr_state == 3'd1)
										score <= score + 1;
									if (posCounter == 2'b01 && curr_state == 3'd2)
										score <= score + 1;
									end
							end
							// position counter
						if (posCounter == 2'b00)
							begin
								x<= 8'd28;
								y <= y + 1;
								tempY <= tempY + 1;
								// ADDED NEW
								tempX <= 8'd28;
								//scoreEnable <= 1'b0;
							
							end
						
						if (posCounter == 2'b01)
							begin
								x<= 8'd68;
								y <= y + 1;
								// ADDED NEW
								tempX <= 8'd68;
								tempY <= tempY + 1;
								//scoreEnable <= 1'b0;
							
							end
							
						if (posCounter == 2'b10)
							begin
								x<= 8'd108;
								y <= y + 1;
								tempY <= tempY + 1;
								// ADDED NEW
								tempX <= 8'd108;
								//scoreEnable <= 1'b0;
							
							end

					
					end
			
			end
	
	
	
	end



endmodule 



///////////// RATE DIVIDER ////////////////

module RateDivider (
input ClockIn,
input Reset,
input [25:0] PULSES,
output Enable
);


// start by setting counter 
	reg [0:100] RateCounter;

	always @(posedge ClockIn) 
		begin
			if (~Reset) 
				begin
					RateCounter <= 0; // check this
				end
			else
			begin
				if (RateCounter > 0)
					begin
						//w <= 1;
						// decreasing the counter by one each time
						RateCounter <= RateCounter - 1;
					end
				else
					begin
						//GENERATE ENABLE PULSE HERE
						//Enable <= (w == 'b0) ? 1'b1:1'b0;
						RateCounter <= PULSES;
						
					end			
			end
		
		end
		
		// assigning enabler depending on situation
		
		assign Enable = (RateCounter == 'b0) ? 1'b1:1'b0;


endmodule 
