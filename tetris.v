module tetris(						
		CLOCK_50,					
		KEY,
		SW,
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,					//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]
		HEX0,		
		HEX1,
		HEX4,
		HEX5,
	);

	input	CLOCK_50;		//	50 MHz
	input [9:0]   SW;
	input [3:0]   KEY;

	output VGA_CLK;  
	output VGA_HS;					
	output VGA_VS;					
	output VGA_BLANK_N;			
	output VGA_SYNC_N;				
	output [9:0]	VGA_R;  
	output [9:0]	VGA_G;
	output [9:0]	VGA_B;   				
	output [6:0] HEX0;		// HEX display for score
	output [6:0] HEX1;		// HEX display for score
	output [6:0] HEX4;		// HEX display for highscore
	output [6:0] HEX5;		// HEX display for highscore
	
	// Reset control, used to start game in INIT state
	wire resetn;
	assign resetn = SW[9];
	
	// Array to store game grid
	reg [0:2] grid [87:0];
	
	// Inputs for VGA 
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	reg [7:0] x_in; // used to autoload x coordinates of blocks
	reg [7:0] y_in; // used to autoload y coordinates of blocks
	reg [2:0] colour_in;
	reg writeEn;
	
	// Counters used to redraw grid
	integer count = 0; 
	reg [27:0] RateDivider;
	
	// Variables used to generate block 
	reg [1:0] block; 
	integer random_count;
	
	// Score variables
	reg [7:0] score;
	wire [7:0] score_out;
	wire [7:0] formatted_score;
	
	// High score variables 
	reg [7:0] high_score;
	wire [7:0] high_score_out;
	wire [7:0] formatted_highscore;
	 
	integer curr_index; //Keeps track of the current block's position (top left square of block)
					    //set to -1 until ready for next block
	
	// Wires for key debouncer modules
	wire left;		//KEY[2] enable
	wire no_left;	//KEY[2] disabled
	wire right;		//KEY[1] enabled
	wire no_right;	//KEY[1] disabled
	wire drop;		//KEY[0] enabled
	wire no_drop;	//KEY[0] disable
	 
	//Row checks used in CHECK_ROWS
	reg lose_case; 
	reg row1;
	reg row2;
    reg row3;
	reg row4;
	reg row5;
	reg row6;
	reg row7;
	reg row8;
	reg row9;
	reg row10; 
	 	
	// Create an Instance of a VGA controller 
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
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
		defparam VGA.BACKGROUND_IMAGE = "bck1.mif";
	
	// Create an instance of the datapath
	datapath D0(.clk(CLOCK_50),
					.resetn(resetn),
					.x(x),
					.y(y),
					.x_in(x_in),
					.y_in(y_in),
					.colour(colour),
					.color_in(colour_in)
					);
	
	// HEX displays for score
	HEX_seg hex0(.c0(formatted_score[0]), .c1(formatted_score[1]), .c2(formatted_score[2]), .c3(formatted_score[3]), .seg(HEX0));
	HEX_seg hex1(.c0(formatted_score[4]), .c1(formatted_score[5]), .c2(formatted_score[6]), .c3(formatted_score[7]), .seg(HEX1));
	
	// Format score, convert hexadecimal to decimal
	format_score format_score1(score_out, formatted_score);
	
	// HEX displays for high score
	HEX_seg hex4(.c0(formatted_highscore[0]), .c1(formatted_highscore[1]), .c2(formatted_highscore[2]), .c3(formatted_highscore[3]), .seg(HEX4));
	HEX_seg hex5(.c0(formatted_highscore[4]), .c1(formatted_highscore[5]), .c2(formatted_highscore[6]), .c3(formatted_highscore[7]), .seg(HEX5));
	
	// Format high score
	format_score format_high_score(high_score_out, formatted_highscore);
	 
	//Debouncer module calls
	debouncer move_left(.raw(KEY[2]), .clk(CLOCK_50), .enabled(left), .disabled(no_left));
	debouncer move_right(.raw(KEY[1]), .clk(CLOCK_50), .enabled(right), .disabled(no_right));
	debouncer drop_piece(.raw(KEY[0]), .clk(CLOCK_50), .enabled(drop), .disabled(no_drop));
	
	// State,FSM values
	reg [2:0] current_state, next_state;
	localparam  INIT        = 3'b000,
               GENERATE    = 3'b001,
               MOVE        = 3'b010,
					DROP        = 3'b011,
               CHECK_ROWS  = 3'b100,
               LOSE        = 3'b101;
	
	// Block types	
	localparam 
				  V_BAR = 2'b00, 	//Vertical Bar
				  SQ = 2'b01,		//Square
				  H_BAR = 2'b10,	//Horizontal Bar
				  L = 2'b11; 		//L Shape
				  
	// Colours for each block
	localparam
				V_BAR_COL = 3'b110,
				SQ_COL = 3'b001,
				H_BAR_COL = 3'b010,
				L_COL = 3'b100; 
	
	// Counter to redraw grid
	always @(posedge CLOCK_50) 
	begin
		if (resetn == 1'b0) //reset counter, have to reload rate
			RateDivider <= 0;
		else if (RateDivider == 0) 
			RateDivider <= 199;
		else
			RateDivider <= RateDivider - 1'b1; //Decrement RateDivider	
	end
	assign Enable = (RateDivider == 0) ? 1 : 0; 
	
	// Random number counter, assigns value to block only when in GENERATE state
	// Ranges from 0-3
	always@(posedge CLOCK_50)
			begin
				if (random_count == 3) //If random_count is at max value(3), reset to 0
					 random_count <= 0;
				else
					random_count<= random_count + 1;
			end 
	
	// FSM 
	always@(posedge CLOCK_50)
	begin 
		case (current_state)
			INIT:
				/*
					Initialize the game, setting game grid to black and resetting score.
				*/
				begin
					// Erase current block if it exists
					grid[curr_index] <= 3'b000;
					grid[curr_index +1] <= 3'b000;
					grid[curr_index +8] <= 3'b000;
					grid[curr_index + 9] <= 3'b000;

					score <= 0; // reset score
					
					// Reset grid, manually setting each element to black
					grid[0] <= 3'b000;
					grid[1] <= 3'b000;
					grid[2] <= 3'b000;
					grid[3] <= 3'b000;
					grid[4] <= 3'b000;
					grid[5] <= 3'b000;
					grid[6] <= 3'b000;
					grid[7] <= 3'b000;
					grid[8] <= 3'b000;
					grid[9] <= 3'b000;
					grid[10] <= 3'b000;
					grid[11] <= 3'b000;
					grid[12] <= 3'b000;
					grid[13] <= 3'b000;
					grid[14] <= 3'b000;
					grid[15] <= 3'b000;
					grid[16] <= 3'b000;
					grid[17] <= 3'b000;
					grid[18] <= 3'b000;
					grid[19] <= 3'b000;
					grid[20] <= 3'b000;
					grid[21] <= 3'b000;
					grid[22] <= 3'b000;
					grid[23] <= 3'b000;
					grid[24] <= 3'b000;
					grid[25] <= 3'b000;
					grid[26] <= 3'b000;
					grid[27] <= 3'b000;
					grid[28] <= 3'b000;
					grid[29] <= 3'b000;
					grid[30] <= 3'b000;
					grid[31] <= 3'b000;
					grid[32] <= 3'b000;
					grid[33] <= 3'b000;
					grid[34] <= 3'b000;
					grid[35] <= 3'b000;
					grid[36] <= 3'b000;
					grid[37] <= 3'b000;
					grid[38] <= 3'b000;
					grid[39] <= 3'b000;
					grid[40] <= 3'b000;
					grid[41] <= 3'b000;
					grid[42] <= 3'b000;
					grid[43] <= 3'b000;
					grid[44] <= 3'b000;
					grid[45] <= 3'b000;
					grid[46] <= 3'b000;
					grid[47] <= 3'b000;
					grid[48] <= 3'b000;
					grid[49] <= 3'b000;
					grid[50] <= 3'b000;
					grid[51] <= 3'b000;
					grid[52] <= 3'b000;
					grid[53] <= 3'b000;
					grid[54] <= 3'b000;
					grid[55] <= 3'b000;
					grid[56] <= 3'b000;
					grid[57] <= 3'b000;
					grid[58] <= 3'b000;
					grid[59] <= 3'b000;
					grid[60] <= 3'b000;
					grid[61] <= 3'b000;
					grid[62] <= 3'b000;
					grid[63] <= 3'b000;
					grid[64] <= 3'b000;
					grid[65] <= 3'b000;
					grid[66] <= 3'b000;
					grid[67] <= 3'b000;
					grid[68] <= 3'b000;
					grid[69] <= 3'b000;
					grid[70] <= 3'b000;
					grid[71] <= 3'b000;
					grid[72] <= 3'b000;
					grid[73] <= 3'b000;
					grid[74] <= 3'b000;
					grid[75] <= 3'b000;
					grid[76] <= 3'b000;
					grid[77] <= 3'b000;
					grid[78] <= 3'b000;
					grid[79] <= 3'b000;
					grid[80] <= 3'b000;
					grid[81] <= 3'b000;
					grid[82] <= 3'b000;
					grid[83] <= 3'b000;
					grid[84] <= 3'b000;
					grid[85] <= 3'b000;
					grid[86] <= 3'b000;
					grid[87] <= 3'b000;
					
					if (resetn == 1'b1) // If SW[9] is on, start the game
						begin
							curr_index <= 3; // Initialize current index to starting position for block generation
							next_state <= GENERATE; 
						end
					else
						next_state <= INIT; // stay in INIT
				end //end of INIT
				
			GENERATE: 
					/*	
						Assign random value from 0-3 to block and go to MOVE state
					*/
				begin
					 
					 // Generate random block
					 block <= random_count;
					 
					 // Set curr_index to generate position
					 curr_index <= 3;
					 
					 next_state <= MOVE;
				end //end of GENERATE
						
			MOVE: 
				/*
					Move the block based on user input, checking if the move is valid
					KEY[2] to move left
					KEY[1] to move right
					Once User presses KEY[0] the block is dropped, moving to DROP state 
				*/
				begin
					if (curr_index != -1) //Check if current block is meant to be moved (to avoid movement when changing states)
						begin
							case(block)
							V_BAR:
								begin 
									if (left == 1'b1 && curr_index != 0 && grid[curr_index - 1] == 3'b000 && grid[curr_index + 7] == 3'b000) //Check if V_BAR can be moved left
										begin
											//Move left
											curr_index <= curr_index - 1;
											
											//Erase previously drawn V_BAR
											grid[curr_index] <= 3'b000;		
											grid[curr_index + 8] <= 3'b000;
										end
									else if (right == 1'b1 && curr_index != 7 && grid[curr_index + 1] == 3'b000 && grid[curr_index + 9] == 3'b000) //Check if V_BAR can be moved right
										begin
											//Move right
											curr_index <= curr_index + 1;
											
											//Erase previously drawn V_BAR
											grid[curr_index - 1] <= 3'b000;
											grid[curr_index] <= 3'b000;
											grid[curr_index +7] <= 3'b000;
											grid[curr_index + 8] <= 3'b000;
										end
									else
										begin
											//If no movement key is pressed, draw V_BAR at current position
											grid[curr_index] <= V_BAR_COL;
											grid[curr_index + 8] <= V_BAR_COL;
										end
								end
							
							SQ: 
								begin 
									if (left == 1'b1 && curr_index != 0 && grid[curr_index - 1] == 3'b000 && grid[curr_index + 7] == 3'b000) //Check if SQ can be moved left
										begin
											//Move left
											curr_index <= curr_index - 1;
											
											//Erase previously drawn SQ
											grid[curr_index +1] <= 3'b000;
											grid[curr_index +2] <= 3'b000;
											grid[curr_index +9] <= 3'b000;
											grid[curr_index + 10] <= 3'b000;
										end
									else if (right == 1'b1 && curr_index != 6 && grid[curr_index + 2] == 3'b000 && grid[curr_index + 10] == 3'b000) //Check if SQ can be moved right
										begin
											//Move right
											curr_index <= curr_index + 1;
											
											//Erase previously drawn SQ
											grid[curr_index - 1] <= 3'b000;
											grid[curr_index] <= 3'b000;
											grid[curr_index +7] <= 3'b000;
											grid[curr_index + 8] <= 3'b000;
										end
									else
										begin
											//if no movement key is pressed, draw SQ at current position
											grid[curr_index] <= SQ_COL;
											grid[curr_index +1] <= SQ_COL;
											grid[curr_index +8] <= SQ_COL;
											grid[curr_index + 9] <= SQ_COL;
										end
								end
						
							H_BAR: 
								begin 
									if (left == 1'b1 && curr_index != 0 && grid[curr_index - 1] == 3'b000) //Check if H_BAR can be moved left
										begin
											//Move left
											curr_index <= curr_index - 1;
											
											//Erase previously drawn H_BAR
											grid[curr_index +1] <= 3'b000;
											grid[curr_index +2] <= 3'b000;
										end
									else if (right == 1'b1 && curr_index != 6 && grid[curr_index + 2] == 3'b000) //Check if H_BAR can be moved right
										begin
											//Move right
											curr_index <= curr_index + 1;
											
											//Erase previously drawn H_BAR
											grid[curr_index - 1] <= 3'b000;
											grid[curr_index] <= 3'b000;
										end
									else
										begin
											//If no movement key is pressed, draw H_BAR at current position
											grid[curr_index] <= H_BAR_COL;
											grid[curr_index+1] <= H_BAR_COL;
										end
									end
							L: 
								begin 
									if (left == 1'b1 && curr_index != 0 && grid[curr_index -1] == 3'b000 && grid[curr_index + 7] == 3'b000) //check if L can be moved left
										begin
											// Move left
											curr_index <= curr_index - 1;
											
											//Erase previously drawn L
											grid[curr_index +1] <= 3'b000;
											grid[curr_index +2] <= 3'b000;
											grid[curr_index +9] <= 3'b000;
											grid[curr_index + 10] <= 3'b000;
										end
									else if (right == 1'b1 && curr_index != 6 && grid[curr_index + 2] == 3'b000 && grid[curr_index + 10] == 3'b000) //check if L can be moved right
										begin
											// Move right
											curr_index <= curr_index + 1;
											
											//Erase previously drawn L
											grid[curr_index - 1] <= 3'b000;
											grid[curr_index] <= 3'b000;
											grid[curr_index +7] <= 3'b000;
											grid[curr_index + 8] <= 3'b000;
										end
									else
										begin
											//if no movement key is pressed, draw L at current postion
											grid[curr_index] <= L_COL;
											grid[curr_index + 1] <= 3'b000;
											grid[curr_index + 8] <= L_COL;
											grid[curr_index + 9] <= L_COL;
										end
								end				
							endcase // end of case(block)
					
						if (drop == 1'b1)
							next_state <= DROP; //Once player presses KEY[0], move to DROP
						else 
							next_state <= MOVE; //Loop in MOVE state until KEY[0] is pressed 
							
					end // end of if (curr_index != -1)
				end //end of MOVE
			
		DROP:
			begin
				/* 
					Loop in DOWN state, incrementing curr_index by 8 (down one row) until a collision occurs 
					the block hits the bottom of the grid 
				
				*/
					if (curr_index != -1) //Check if current block is meant to be moved (to avoid movement when changing states)
					begin
						case(block)
							V_BAR:
								begin
									if (curr_index + 8 < 80 && grid[curr_index + 16] == 3'b000) //Check if V_BAR can be moved down 
										begin
											//Move V_BAR's position down one row
											curr_index <= curr_index + 8;
											
											//Erase previous V_BAR
											grid[curr_index - 8] <= 3'b000;
											grid[curr_index] <= 3'b000;
											
											next_state <= DROP; //loop in DROP
										end
									else
										begin 
											// Collision occured, draw V_BAR at new position
											grid[curr_index] <= V_BAR_COL;
											grid[curr_index +8] <= V_BAR_COL;
											
											next_state <= CHECK_ROWS; //move to CHECK_ROWS
										 end
								end 
							SQ:
								begin
									if (curr_index + 8 < 79 && grid[curr_index + 16] == 3'b000 && grid[curr_index + 17] == 3'b000) //Check if SQ can be moved down
										begin
											//Move SQ's position down one row
											curr_index <= curr_index + 8;
											
											//Erase previous SQ
											grid[curr_index - 8] <= 3'b000;
											grid[curr_index -7] <= 3'b000;
											grid[curr_index] <= 3'b000;
											grid[curr_index + 1] <= 3'b000;
											
											next_state <= DROP; //loop in DROP
										end
									else
										begin 
											//Collision occured, draw SQ at new position
											grid[curr_index] <= SQ_COL;
											grid[curr_index +1] <= SQ_COL;
											grid[curr_index +8] <= SQ_COL;
											grid[curr_index + 9] <= SQ_COL;
											
											next_state <= CHECK_ROWS; //move to CHECK_ROWS
										 end
								end
							H_BAR:
								begin
									if (curr_index + 8 < 87 && grid[curr_index + 8] == 3'b000 && grid[curr_index + 9] == 3'b000) //Check if H_BAR can be moved down
										begin
											// Move H_BAR's position down one row
											curr_index <= curr_index + 8;
											
											//Erase previous H_BAR
											grid[curr_index] <= 3'b000;
											grid[curr_index + 1] <= 3'b000;
											
											next_state <= DROP; //loop in DROP
										end
									else
										begin 
											//Collision occured, draw H_BAR at new position
											grid[curr_index] <= H_BAR_COL;
											grid[curr_index+1] <= H_BAR_COL;
											
											next_state <= CHECK_ROWS; //move to CHECK_ROWS
										end
								end
							L:
								begin
									if (curr_index + 8 < 79 && grid[curr_index + 16] == 3'b000 && grid[curr_index + 17] == 3'b000) //Check if L can be moved down
										begin
											// Move L's position down one row
											curr_index <= curr_index + 8;
											
											//Erase previous L
											grid[curr_index - 8] <= 3'b000;
											grid[curr_index -7] <= 3'b000;
											grid[curr_index] <= 3'b000;
											grid[curr_index + 1] <= 3'b000;
											
											next_state <= DROP; //loop in DROP
										end
									else
										begin
											//Collision occured, draw L at new position
											grid[curr_index] <= L_COL;
											grid[curr_index + 8] <= L_COL;
											grid[curr_index + 9] <= L_COL;
											
											next_state <= CHECK_ROWS; //move to CHECK_ROWS
										end
								end
						endcase // end of case(block)
					end	// end of if (curr_index != -1)			
			end //end of DROP
			
		CHECK_ROWS:
			begin
				/*
					Check if any of the grid rows are full, if so clear the first full row found and shift everything down. Loop in this state until 
					no full rows remain.
					Shifting is done by manually assigning each row with its new values
				*/
				
				//Checks if anything is in the 2nd top row, if so the game is over
				lose_case = grid[8] != 0 || grid[9] != 0 || grid[10] != 0 || grid[11] != 0 || grid[12] != 0 || grid[13] != 0 || grid[14] != 0 || grid[15] != 0; 
				
				row1 = grid[8] != 0 && grid[9] != 0 && grid[10] != 0 && grid[11] != 0 && grid[12] != 0 && grid[13] != 0 && grid[14] != 0 && grid[15] != 0;
				row2 = grid[16] != 0 && grid[17] != 0 && grid[18] != 0 && grid[19] != 0 && grid[20] != 0 && grid[21] != 0 && grid[22] != 0 && grid[23] != 0;
				row3 = grid[24] != 0 && grid[25] != 0 && grid[26] != 0 && grid[27] != 0 && grid[28] != 0 && grid[29] != 0 && grid[30] != 0 && grid[31] != 0;
				row4 = grid[32] != 0 && grid[33] != 0 && grid[34] != 0 && grid[35] != 0 && grid[36] != 0 && grid[37] != 0 && grid[38] != 0 && grid[39] != 0;
				row5 = grid[40] != 0 && grid[41] != 0 && grid[42] != 0 && grid[43] != 0 && grid[44] != 0 && grid[45] != 0 && grid[46] != 0 && grid[47] != 0;
				row6 = grid[48] != 0 && grid[49] != 0 && grid[50] != 0 && grid[51] != 0 && grid[52] != 0 && grid[53] != 0 && grid[54] != 0 && grid[55] != 0;
				row7 = grid[56] != 0 && grid[57] != 0 && grid[58] != 0 && grid[59] != 0 && grid[60] != 0 && grid[61] != 0 && grid[62] != 0 && grid[63] != 0;
				row8 = grid[64] != 0 && grid[65] != 0 && grid[66] != 0 && grid[67] != 0 && grid[68] != 0 && grid[69] != 0 && grid[70] != 0 && grid[71] != 0;
				row9 = grid[72] != 0 && grid[73] != 0 && grid[74] != 0 && grid[75] != 0 && grid[76] != 0 && grid[77] != 0 && grid[78] != 0 && grid[79] != 0;
				row10 = grid[80] != 0 && grid[81] != 0 && grid[82] != 0 && grid[83] != 0 && grid[84] != 0 && grid[85] != 0 && grid[86] != 0 && grid[87] != 0;
			
				// Check rows from top to bottom
					if (row1 == 1'b1)
						begin
							score = score + 1; //increment score
							
							//row1 <= top row
							grid[8] <= grid[0];
							grid[9] <= grid[1];
							grid[10] <= grid[2];
							grid[11] <= grid[3];
							grid[12] <= grid[4];
							grid[13] <= grid[5];
							grid[14] <= grid[6];
							grid[15] <= grid[7];
							
							//erase top row
							grid[0] <= 3'b000;
							grid[1] <= 3'b000;
							grid[2] <= 3'b000;
							grid[3] <= 3'b000;
							grid[4] <= 3'b000;
							grid[5] <= 3'b000;
							grid[6] <= 3'b000;
							grid[7] <= 3'b000;	
						end 
						
					else if (row2 == 1'b1)
						begin
							score = score + 1; //increment score
							
							//row2 <= row1	
							grid[16] <= grid[8];
							grid[17] <= grid[9];
							grid[18] <= grid[10];
							grid[19] <= grid[11];
							grid[20] <= grid[12];
							grid[21] <= grid[13];
							grid[22] <= grid[14];
							
							//row1 <= top row
							grid[8] <= grid[0];
							grid[9] <= grid[1];
							grid[10] <= grid[2];
							grid[11] <= grid[3];
							grid[12] <= grid[4];
							grid[13] <= grid[5];
							grid[14] <= grid[6];
							grid[15] <= grid[7];
							
							//erase top row
							grid[0] <= 3'b000;
							grid[1] <= 3'b000;
							grid[2] <= 3'b000;
							grid[3] <= 3'b000;
							grid[4] <= 3'b000;
							grid[5] <= 3'b000;
							grid[6] <= 3'b000;
							grid[7] <= 3'b000;
						end
						
					else if (row3 == 1'b1)
						begin
							score = score + 1; //increment score
							
							//row3 <= row2
							grid[24] <= grid[16];
							grid[25] <= grid[17];
							grid[26] <= grid[18];
							grid[27] <= grid[19];
							grid[28] <= grid[20];
							grid[29] <= grid[21];
							grid[30] <= grid[22];
							grid[31] <= grid[23];
							
							//row2 <= row1	
							grid[16] <= grid[8];
							grid[17] <= grid[9];
							grid[18] <= grid[10];
							grid[19] <= grid[11];
							grid[20] <= grid[12];
							grid[21] <= grid[13];
							grid[22] <= grid[14];
							
							//row1 <= top row
							grid[8] <= grid[0];
							grid[9] <= grid[1];
							grid[10] <= grid[2];
							grid[11] <= grid[3];
							grid[12] <= grid[4];
							grid[13] <= grid[5];
							grid[14] <= grid[6];
							grid[15] <= grid[7];
							
							//erase top row
							grid[0] <= 3'b000;
							grid[1] <= 3'b000;
							grid[2] <= 3'b000;
							grid[3] <= 3'b000;
							grid[4] <= 3'b000;
							grid[5] <= 3'b000;
							grid[6] <= 3'b000;
							grid[7] <= 3'b000;
						end
						
					else if (row4 == 1'b1)
						begin
							score = score + 1; //increment score
							
							//row4 <= row3
							grid[32] <= grid[24];
							grid[33] <= grid[25];
							grid[34] <= grid[26];
							grid[35] <= grid[27];
							grid[36] <= grid[28];
							grid[37] <= grid[29];
							grid[38] <= grid[30];
							grid[39] <= grid[31];
							
							//row3 <= row2
							grid[24] <= grid[16];
							grid[25] <= grid[17];
							grid[26] <= grid[18];
							grid[27] <= grid[19];
							grid[28] <= grid[20];
							grid[29] <= grid[21];
							grid[30] <= grid[22];
							grid[31] <= grid[23];
							
							//row2 <= row1	
							grid[16] <= grid[8];
							grid[17] <= grid[9];
							grid[18] <= grid[10];
							grid[19] <= grid[11];
							grid[20] <= grid[12];
							grid[21] <= grid[13];
							grid[22] <= grid[14];
							
							//row1 <= top row
							grid[8] <= grid[0];
							grid[9] <= grid[1];
							grid[10] <= grid[2];
							grid[11] <= grid[3];
							grid[12] <= grid[4];
							grid[13] <= grid[5];
							grid[14] <= grid[6];
							grid[15] <= grid[7];
							
							//erase top row
							grid[0] <= 3'b000;
							grid[1] <= 3'b000;
							grid[2] <= 3'b000;
							grid[3] <= 3'b000;
							grid[4] <= 3'b000;
							grid[5] <= 3'b000;
							grid[6] <= 3'b000;
							grid[7] <= 3'b000;
						end
						
					else if (row5 == 1'b1)
						begin
							score = score + 1; //increment score
							
							//row5 <= row4
							grid[40] <= grid[32];
							grid[41] <= grid[33];
							grid[42] <= grid[34];
							grid[43] <= grid[35];
							grid[44] <= grid[36];
							grid[45] <= grid[37];
							grid[46] <= grid[38];
							grid[47] <= grid[39];
							
							//row4 <= row3
							grid[32] <= grid[24];
							grid[33] <= grid[25];
							grid[34] <= grid[26];
							grid[35] <= grid[27];
							grid[36] <= grid[28];
							grid[37] <= grid[29];
							grid[38] <= grid[30];
							grid[39] <= grid[31];
							
							//row3 <= row2
							grid[24] <= grid[16];
							grid[25] <= grid[17];
							grid[26] <= grid[18];
							grid[27] <= grid[19];
							grid[28] <= grid[20];
							grid[29] <= grid[21];
							grid[30] <= grid[22];
							grid[31] <= grid[23];
							
							//row2 <= row1	
							grid[16] <= grid[8];
							grid[17] <= grid[9];
							grid[18] <= grid[10];
							grid[19] <= grid[11];
							grid[20] <= grid[12];
							grid[21] <= grid[13];
							grid[22] <= grid[14];
							
							//row1 <= top row
							grid[8] <= grid[0];
							grid[9] <= grid[1];
							grid[10] <= grid[2];
							grid[11] <= grid[3];
							grid[12] <= grid[4];
							grid[13] <= grid[5];
							grid[14] <= grid[6];
							grid[15] <= grid[7];
							
							//erase top row
							grid[0] <= 3'b000;
							grid[1] <= 3'b000;
							grid[2] <= 3'b000;
							grid[3] <= 3'b000;
							grid[4] <= 3'b000;
							grid[5] <= 3'b000;
							grid[6] <= 3'b000;
							grid[7] <= 3'b000;
						end
						
					else if (row6 == 1'b1)
						begin
							score = score + 1; //increment score
							
							//row6 <= row5
							grid[48] <= grid[40];
							grid[49] <= grid[41];
							grid[50] <= grid[42];
							grid[51] <= grid[43];
							grid[52] <= grid[44];
							grid[53] <= grid[45];
							grid[54] <= grid[46];
							grid[55] <= grid[47];
							
							//row5 <= row4
							grid[40] <= grid[32];
							grid[41] <= grid[33];
							grid[42] <= grid[34];
							grid[43] <= grid[35];
							grid[44] <= grid[36];
							grid[45] <= grid[37];
							grid[46] <= grid[38];
							grid[47] <= grid[39];
							
							//row4 <= row3
							grid[32] <= grid[24];
							grid[33] <= grid[25];
							grid[34] <= grid[26];
							grid[35] <= grid[27];
							grid[36] <= grid[28];
							grid[37] <= grid[29];
							grid[38] <= grid[30];
							grid[39] <= grid[31];
							
							//row3 <= row2
							grid[24] <= grid[16];
							grid[25] <= grid[17];
							grid[26] <= grid[18];
							grid[27] <= grid[19];
							grid[28] <= grid[20];
							grid[29] <= grid[21];
							grid[30] <= grid[22];
							grid[31] <= grid[23];
							
							//row2 <= row1	
							grid[16] <= grid[8];
							grid[17] <= grid[9];
							grid[18] <= grid[10];
							grid[19] <= grid[11];
							grid[20] <= grid[12];
							grid[21] <= grid[13];
							grid[22] <= grid[14];
							
							//row1 <= top row
							grid[8] <= grid[0];
							grid[9] <= grid[1];
							grid[10] <= grid[2];
							grid[11] <= grid[3];
							grid[12] <= grid[4];
							grid[13] <= grid[5];
							grid[14] <= grid[6];
							grid[15] <= grid[7];
							
							//erase top row
							grid[0] <= 3'b000;
							grid[1] <= 3'b000;
							grid[2] <= 3'b000;
							grid[3] <= 3'b000;
							grid[4] <= 3'b000;
							grid[5] <= 3'b000;
							grid[6] <= 3'b000;
							grid[7] <= 3'b000;
						end
						
					else if (row7 == 1'b1)
						begin
							score = score + 1; //increment score
							
							//row7 <= row6
							grid[56] <= grid[48];
							grid[57] <= grid[49];
							grid[58] <= grid[50];
							grid[59] <= grid[51];
							grid[60] <= grid[52];
							grid[61] <= grid[53];
							grid[62] <= grid[54];
							grid[63] <= grid[55];
							
							//row6 <= row5
							grid[48] <= grid[40];
							grid[49] <= grid[41];
							grid[50] <= grid[42];
							grid[51] <= grid[43];
							grid[52] <= grid[44];
							grid[53] <= grid[45];
							grid[54] <= grid[46];
							grid[55] <= grid[47];
							
							//row5 <= row4
							grid[40] <= grid[32];
							grid[41] <= grid[33];
							grid[42] <= grid[34];
							grid[43] <= grid[35];
							grid[44] <= grid[36];
							grid[45] <= grid[37];
							grid[46] <= grid[38];
							grid[47] <= grid[39];
							
							//row4 <= row3
							grid[32] <= grid[24];
							grid[33] <= grid[25];
							grid[34] <= grid[26];
							grid[35] <= grid[27];
							grid[36] <= grid[28];
							grid[37] <= grid[29];
							grid[38] <= grid[30];
							grid[39] <= grid[31];
							
							//row3 <= row2
							grid[24] <= grid[16];
							grid[25] <= grid[17];
							grid[26] <= grid[18];
							grid[27] <= grid[19];
							grid[28] <= grid[20];
							grid[29] <= grid[21];
							grid[30] <= grid[22];
							grid[31] <= grid[23];
							
							//row2 <= row1	
							grid[16] <= grid[8];
							grid[17] <= grid[9];
							grid[18] <= grid[10];
							grid[19] <= grid[11];
							grid[20] <= grid[12];
							grid[21] <= grid[13];
							grid[22] <= grid[14];
							
							//row1 <= top row
							grid[8] <= grid[0];
							grid[9] <= grid[1];
							grid[10] <= grid[2];
							grid[11] <= grid[3];
							grid[12] <= grid[4];
							grid[13] <= grid[5];
							grid[14] <= grid[6];
							grid[15] <= grid[7];
							
							//erase top row
							grid[0] <= 3'b000;
							grid[1] <= 3'b000;
							grid[2] <= 3'b000;
							grid[3] <= 3'b000;
							grid[4] <= 3'b000;
							grid[5] <= 3'b000;
							grid[6] <= 3'b000;
							grid[7] <= 3'b000;
						end
						
					else if (row8 == 1'b1)
						begin
							score = score + 1; //increment score
							
							//row8 <= row7
							grid[64] <= grid[56];
							grid[65] <= grid[57];
							grid[66] <= grid[58];
							grid[67] <= grid[59];
							grid[68] <= grid[60];
							grid[69] <= grid[61];
							grid[70] <= grid[62];
							grid[71] <= grid[63];
							
							//row7 <= row6
							grid[56] <= grid[48];
							grid[57] <= grid[49];
							grid[58] <= grid[50];
							grid[59] <= grid[51];
							grid[60] <= grid[52];
							grid[61] <= grid[53];
							grid[62] <= grid[54];
							grid[63] <= grid[55];
							
							//row6 <= row5
							grid[48] <= grid[40];
							grid[49] <= grid[41];
							grid[50] <= grid[42];
							grid[51] <= grid[43];
							grid[52] <= grid[44];
							grid[53] <= grid[45];
							grid[54] <= grid[46];
							grid[55] <= grid[47];
							
							//row5 <= row4
							grid[40] <= grid[32];
							grid[41] <= grid[33];
							grid[42] <= grid[34];
							grid[43] <= grid[35];
							grid[44] <= grid[36];
							grid[45] <= grid[37];
							grid[46] <= grid[38];
							grid[47] <= grid[39];
							
							//row4 <= row3
							grid[32] <= grid[24];
							grid[33] <= grid[25];
							grid[34] <= grid[26];
							grid[35] <= grid[27];
							grid[36] <= grid[28];
							grid[37] <= grid[29];
							grid[38] <= grid[30];
							grid[39] <= grid[31];
							
							//row3 <= row2
							grid[24] <= grid[16];
							grid[25] <= grid[17];
							grid[26] <= grid[18];
							grid[27] <= grid[19];
							grid[28] <= grid[20];
							grid[29] <= grid[21];
							grid[30] <= grid[22];
							grid[31] <= grid[23];
							
							//row2 <= row1	
							grid[16] <= grid[8];
							grid[17] <= grid[9];
							grid[18] <= grid[10];
							grid[19] <= grid[11];
							grid[20] <= grid[12];
							grid[21] <= grid[13];
							grid[22] <= grid[14];
							
							//row1 <= top row
							grid[8] <= grid[0];
							grid[9] <= grid[1];
							grid[10] <= grid[2];
							grid[11] <= grid[3];
							grid[12] <= grid[4];
							grid[13] <= grid[5];
							grid[14] <= grid[6];
							grid[15] <= grid[7];
							
							//erase top row
							grid[0] <= 3'b000;
							grid[1] <= 3'b000;
							grid[2] <= 3'b000;
							grid[3] <= 3'b000;
							grid[4] <= 3'b000;
							grid[5] <= 3'b000;
							grid[6] <= 3'b000;
							grid[7] <= 3'b000;
						end
						
					else if (row9 == 1'b1)
						begin
							score = score + 1; //increment score
							
							//row9 <= row8
							grid[72] <= grid[64];
							grid[73] <= grid[65];
							grid[74] <= grid[66];
							grid[75] <= grid[67];
							grid[76] <= grid[68];
							grid[77] <= grid[69];
							grid[78] <= grid[70];
							grid[79] <= grid[71];
							
							
							//row8 <= row7
							grid[64] <= grid[56];
							grid[65] <= grid[57];
							grid[66] <= grid[58];
							grid[67] <= grid[59];
							grid[68] <= grid[60];
							grid[69] <= grid[61];
							grid[70] <= grid[62];
							grid[71] <= grid[63];
							
							//row7 <= row6
							grid[56] <= grid[48];
							grid[57] <= grid[49];
							grid[58] <= grid[50];
							grid[59] <= grid[51];
							grid[60] <= grid[52];
							grid[61] <= grid[53];
							grid[62] <= grid[54];
							grid[63] <= grid[55];
							
							//row6 <= row5
							grid[48] <= grid[40];
							grid[49] <= grid[41];
							grid[50] <= grid[42];
							grid[51] <= grid[43];
							grid[52] <= grid[44];
							grid[53] <= grid[45];
							grid[54] <= grid[46];
							grid[55] <= grid[47];
							
							//row5 <= row4
							grid[40] <= grid[32];
							grid[41] <= grid[33];
							grid[42] <= grid[34];
							grid[43] <= grid[35];
							grid[44] <= grid[36];
							grid[45] <= grid[37];
							grid[46] <= grid[38];
							grid[47] <= grid[39];
							
							//row4 <= row3
							grid[32] <= grid[24];
							grid[33] <= grid[25];
							grid[34] <= grid[26];
							grid[35] <= grid[27];
							grid[36] <= grid[28];
							grid[37] <= grid[29];
							grid[38] <= grid[30];
							grid[39] <= grid[31];
							
							//row3 <= row2
							grid[24] <= grid[16];
							grid[25] <= grid[17];
							grid[26] <= grid[18];
							grid[27] <= grid[19];
							grid[28] <= grid[20];
							grid[29] <= grid[21];
							grid[30] <= grid[22];
							grid[31] <= grid[23];
							
							//row2 <= row1	
							grid[16] <= grid[8];
							grid[17] <= grid[9];
							grid[18] <= grid[10];
							grid[19] <= grid[11];
							grid[20] <= grid[12];
							grid[21] <= grid[13];
							grid[22] <= grid[14];
							
							//row1 <= top row
							grid[8] <= grid[0];
							grid[9] <= grid[1];
							grid[10] <= grid[2];
							grid[11] <= grid[3];
							grid[12] <= grid[4];
							grid[13] <= grid[5];
							grid[14] <= grid[6];
							grid[15] <= grid[7];
							
							//erase top row
							grid[0] <= 3'b000;
							grid[1] <= 3'b000;
							grid[2] <= 3'b000;
							grid[3] <= 3'b000;
							grid[4] <= 3'b000;
							grid[5] <= 3'b000;
							grid[6] <= 3'b000;
							grid[7] <= 3'b000;
						end
						
					else if (row10 == 1'b1)
						begin
							score = score + 1; //increment score
							
							grid[80] <= 3'b000;
							grid[81] <= 3'b000;
							grid[82] <= 3'b000;
							grid[83] <= 3'b000;
							grid[84] <= 3'b000;
							grid[85] <= 3'b000;
							grid[86] <= 3'b000;
							grid[87] <= 3'b000;
							
							//row10 <= row9
							grid[80] <= grid[72];
							grid[81] <= grid[73];
							grid[82] <= grid[74];
							grid[83] <= grid[75];
							grid[84] <= grid[76];
							grid[85] <= grid[77];
							grid[86] <= grid[78];
							grid[87] <= grid[79];
							
							//row9 <= row8
							grid[72] <= grid[64];
							grid[73] <= grid[65];
							grid[74] <= grid[66];
							grid[75] <= grid[67];
							grid[76] <= grid[68];
							grid[77] <= grid[69];
							grid[78] <= grid[70];
							grid[79] <= grid[71];
							
							
							//row8 <= row7
							grid[64] <= grid[56];
							grid[65] <= grid[57];
							grid[66] <= grid[58];
							grid[67] <= grid[59];
							grid[68] <= grid[60];
							grid[69] <= grid[61];
							grid[70] <= grid[62];
							grid[71] <= grid[63];
							
							//row7 <= row6
							grid[56] <= grid[48];
							grid[57] <= grid[49];
							grid[58] <= grid[50];
							grid[59] <= grid[51];
							grid[60] <= grid[52];
							grid[61] <= grid[53];
							grid[62] <= grid[54];
							grid[63] <= grid[55];
							
							//row6 <= row5
							grid[48] <= grid[40];
							grid[49] <= grid[41];
							grid[50] <= grid[42];
							grid[51] <= grid[43];
							grid[52] <= grid[44];
							grid[53] <= grid[45];
							grid[54] <= grid[46];
							grid[55] <= grid[47];
							
							//row5 <= row4
							grid[40] <= grid[32];
							grid[41] <= grid[33];
							grid[42] <= grid[34];
							grid[43] <= grid[35];
							grid[44] <= grid[36];
							grid[45] <= grid[37];
							grid[46] <= grid[38];
							grid[47] <= grid[39];
							
							//row4 <= row3
							grid[32] <= grid[24];
							grid[33] <= grid[25];
							grid[34] <= grid[26];
							grid[35] <= grid[27];
							grid[36] <= grid[28];
							grid[37] <= grid[29];
							grid[38] <= grid[30];
							grid[39] <= grid[31];
							
							//row3 <= row2
							grid[24] <= grid[16];
							grid[25] <= grid[17];
							grid[26] <= grid[18];
							grid[27] <= grid[19];
							grid[28] <= grid[20];
							grid[29] <= grid[21];
							grid[30] <= grid[22];
							grid[31] <= grid[23];
							
							//row2 <= row1	
							grid[16] <= grid[8];
							grid[17] <= grid[9];
							grid[18] <= grid[10];
							grid[19] <= grid[11];
							grid[20] <= grid[12];
							grid[21] <= grid[13];
							grid[22] <= grid[14];
							
							//row1 <= top row
							grid[8] <= grid[0];
							grid[9] <= grid[1];
							grid[10] <= grid[2];
							grid[11] <= grid[3];
							grid[12] <= grid[4];
							grid[13] <= grid[5];
							grid[14] <= grid[6];
							grid[15] <= grid[7];
							
							//erase top row
							grid[0] <= 3'b000;
							grid[1] <= 3'b000;
							grid[2] <= 3'b000;
							grid[3] <= 3'b000;
							grid[4] <= 3'b000;
							grid[5] <= 3'b000;
							grid[6] <= 3'b000;
							grid[7] <= 3'b000;
						end
				
				
				if (lose_case != 0) //Check if player has lost the game
					next_state <= LOSE;
				else if (row1 + row2 + row3 + row4 + row5 + row6 + row7 + row8 + row9 + row10 == 0) // Check if there are any completed rows remaining
					begin
						next_state <= GENERATE; //No complete rows, generate a new block
					end
				else
					begin
						curr_index <= -1; //Set curr_index to -1 to prevent unwanted block movement
						next_state <= CHECK_ROWS; //Loop back to CHECK_ROWS to continue clearing rows
					end
			end
			
		LOSE:
			begin
				/* 
					Outputs lose screen and checks if player's current score is a new highscore. If so
					store the current score in the highscore register
					User can restart the game by turning on SW[8]
				*/
				curr_index <= -1; // set curr_index to -1 to prevent unwanted block movement 
				
					// Draw the lose screen by manually assigning grid values 
					grid[0] <= 3'b000;
					grid[1] <= 3'b000;
					grid[2] <= 3'b000;
					grid[3] <= 3'b000;
					grid[4] <= 3'b000;
					grid[5] <= 3'b000;
					grid[6] <= 3'b000;
					grid[7] <= 3'b000;
					grid[8] <= 3'b000;
					grid[9] <= 3'b000;
					grid[10] <= 3'b000;
					grid[11] <= 3'b000;
					grid[12] <= 3'b000;
					grid[13] <= 3'b000;
					grid[14] <= 3'b000;
					grid[15] <= 3'b000;
					grid[16] <= 3'b000;
					grid[17] <= 3'b000;
					grid[18] <= 3'b011;
					grid[19] <= 3'b011;
					grid[20] <= 3'b000;
					grid[21] <= 3'b000;
					grid[22] <= 3'b000;
					grid[23] <= 3'b000;
					grid[24] <= 3'b000;
					grid[25] <= 3'b000;
					grid[26] <= 3'b011;
					grid[27] <= 3'b011;
					grid[28] <= 3'b000;
					grid[29] <= 3'b000;
					grid[30] <= 3'b000;
					grid[31] <= 3'b000;
					grid[32] <= 3'b000;
					grid[33] <= 3'b000;
					grid[34] <= 3'b011;
					grid[35] <= 3'b011;
					grid[36] <= 3'b000;
					grid[37] <= 3'b000;
					grid[38] <= 3'b000;
					grid[39] <= 3'b000;
					grid[40] <= 3'b000;
					grid[41] <= 3'b000;
					grid[42] <= 3'b011;
					grid[43] <= 3'b011;
					grid[44] <= 3'b000;
					grid[45] <= 3'b000;
					grid[46] <= 3'b000;
					grid[47] <= 3'b000;
					grid[48] <= 3'b000;
					grid[49] <= 3'b000;
					grid[50] <= 3'b011;
					grid[51] <= 3'b011;
					grid[52] <= 3'b000;
					grid[53] <= 3'b000;
					grid[54] <= 3'b000;
					grid[55] <= 3'b000;
					grid[56] <= 3'b000;
					grid[57] <= 3'b000;
					grid[58] <= 3'b011;
					grid[59] <= 3'b011;
					grid[60] <= 3'b011;
					grid[61] <= 3'b011;
					grid[62] <= 3'b011;
					grid[63] <= 3'b000;
					grid[64] <= 3'b000;
					grid[65] <= 3'b000;
					grid[66] <= 3'b011;
					grid[67] <= 3'b011;
					grid[68] <= 3'b011;
					grid[69] <= 3'b011;
					grid[70] <= 3'b011;
					grid[71] <= 3'b000;
					grid[72] <= 3'b000;
					grid[73] <= 3'b000;
					grid[74] <= 3'b000;
					grid[75] <= 3'b000;
					grid[76] <= 3'b000;
					grid[77] <= 3'b000;
					grid[78] <= 3'b000;
					grid[79] <= 3'b000;
					grid[80] <= 3'b000;
					grid[81] <= 3'b000;
					grid[82] <= 3'b000;
					grid[83] <= 3'b000;
					grid[84] <= 3'b000;
					grid[85] <= 3'b000;
					grid[86] <= 3'b000;
					grid[87] <= 3'b000;
				
				// Check if score is the new highscore
				if (score > high_score)
						begin
							high_score <= score; //store the new highscore
						end
					
				//If SW[8] is on, restart the game
				if (SW[8] == 1'b1)
					next_state <= INIT;
				else
					next_state <= LOSE;  //loop in LOSE state until SW[8] is on
					
			end //end of LOSE
		endcase	// end of case(current_state)	
	end //end of FSM
		
	
	//store score values to be outputted to HEX displays
	assign score_out = score;
	assign high_score_out = high_score;
	
	// Update current_state 
    always@(posedge CLOCK_50)
    begin: state_FFs
        if(!resetn)
            current_state <= INIT;
        else
            current_state <= next_state;
    end
	
	//Redrawing game grid
	always@(posedge CLOCK_50)
	begin
		if (Enable == 1'b1)
			begin
				if (count == 0) //First block in grid (top-left corner)
					begin
						x_in <= 34; //initialize x_in to x coordinate of the pixel for the first block
						y_in <= 17;	//initialize y_in to y coordinate of the pixel for the first block
						writeEn <= 1'b1;
					end 
				else if (count%8 == 0 && count < 96) //Reached the end of a row
					begin
						x_in <= 34; //set x_in to x coordinate of the pixel for the first block in the row
						y_in <= y_in + 8; // set y_in to y coordinate of tof the pixel for the first block in next row
					end
				else if (count < 96)
					begin
						x_in <= x_in + 8; //move the x coordinate to the next block
					end 
				
				if (count > 96) // Completed drawing the entire grid 
					count <= 0; //restart count, prepare to draw next state of the grid
				else
					count <= count + 1; // increment count
					
				if (count >= 88)
					colour_in <= 3'b111; //Create the bottom white border
				else
					colour_in <= grid[count]; // assign the pixel colour according to the value of the grid	
			end
    end
endmodule

// HEX display module from Labs
module HEX_seg(c0, c1, c2, c3, seg); 
    input c0; 
    input c1; 
    input c2; 
    input c3; 
    
	output [6:0] seg; 
	
   assign seg[0] = (~c3 & ~c2 & ~c1 & c0) | (~c3 & c2 & ~c1 & ~c0) | (c3 & c2 & ~c1 & c0) | (c3 & ~c2 & c1 & c0);
	assign seg[1] = (~c3 & c2 & ~c1 & c0) | (c2 & c1 & ~c0) | (c3 & c2 & ~c0) | (c3 & c1 & c0);
	assign seg[2] = (~c3 & ~c2 & c1 & ~c0) | (c3 & c2 & ~c0) | (c3 & c2 & c1);
	assign seg[3] = (~c3 & ~c2 & ~c1 & c0) | (~c3 & c2 & ~c1 & ~c0) | (c2 & c1 & c0) | (c3 & ~c2 & c1 & ~c0);
	assign seg[4] = (~c3 & c0) | (~c3 & c2 & ~c1) | (~c2 & ~c1 & c0);
	assign seg[5] = (~c3 & ~c2 & c0) | (~c3 & ~c2 & c1) | (~c3 & c1 & c0) | (c3 & c2 & ~c1 & c0);
	assign seg[6] = (~c3 & ~c2 & ~c1) | (~c3 & c2 & c1 & c0) | (c3 & c2 & ~c1 & ~c0);
endmodule

//VGA datapath module from Labs
module datapath(input clk,
					 input resetn,
					 input [6:0] x_in,
					 input [6:0] y_in,
					 input [2:0] color_in,
					 output [2:0] colour,
					 output [7:0] x,
					 output [7:0] y
					 );
    
	wire y_enable;
	reg [7:0] x_previous;
	reg [6:0] y_previous;
	reg [2:0] color;
    reg [7:0] x_counter, y_counter;
	 
    // Registers x, y, color
    always@(posedge clk) 
		begin
        if(!resetn) begin
            x_previous <= 8'b0;
			  y_previous <= 7'b0;
			color <= 3'b0; // black color
        end
        else 
			begin
            x_previous <= {x_in};
            y_previous <= y_in;
             color <= color_in;
			end
      end
	
    always @(posedge clk) begin
		if (!resetn)
			x_counter <= 3'b000;
		else begin
			if (x_counter == 3'b111)
				x_counter <= 3'b000;
			else
				x_counter <= x_counter + 1'b1;
		end
	end
	
	assign y_enable = (x_counter == 3'b111) ? 1 : 0;

   always @(posedge clk) begin
		if (!resetn)
			y_counter <= 3'b000;
		else if (y_enable) begin
			if (y_counter != 3'b111)
				y_counter <= y_counter + 1'b1;
			else 
				y_counter <= 3'b000;
		end
	end
	
	assign x = x_previous + x_counter;
	assign y = y_previous + y_counter;
	assign colour = color; 
endmodule


/*
	Debouncer Module created by Robert Fotino and Vu Le
	Source code: https://github.com/rfotino/verilog-tetris/blob/master/debouncer.v
*/
module debouncer(
    input wire  raw,
    input wire  clk,
    output wire enabled,
    output wire disabled
    );

    reg debounced;
    reg debounced_prev;
    reg [15:0] counter;

    initial begin
        debounced = 0;
        debounced_prev = 0;
        counter = 0;
    end

    always @ (posedge clk) begin
        // 200 Hz
        if (counter == 12500) begin
            counter <= 0;
            debounced <= raw;
        end else begin
            counter <= counter + 1;
        end

        // Update previous
        debounced_prev <= debounced;
    end

    assign enabled = debounced && !debounced_prev;
    assign disabled = !debounced && debounced_prev;
endmodule 

/*
	Module to format score, converting from hexadecimal to decimal
	Created by Matthew Chau and Zixiong Lin
	Source code: https://github.com/chaumatt/SpaceInvaders/blob/master/format_score.v
 */
module format_score(input [7:0] score, output reg [7:0] formatted);
	always @(*)
	begin
		if (score[3:0] >= 4'b1001) begin
			formatted[3:0] = score[3:0] - 4'b1001;
			formatted = {(score[7:4] + 1), formatted[3:0]};
		end
		else begin
			formatted = score;
		end
	end
endmodule
