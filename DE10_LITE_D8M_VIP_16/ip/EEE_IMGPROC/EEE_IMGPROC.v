module EEE_IMGPROC(
	// global clock & reset
	clk,
	reset_n,
	
	// mm slave
	s_chipselect,
	s_read,
	s_write,
	s_readdata,
	s_writedata,
	s_address,

	// stream sink
	sink_data,
	sink_valid,
	sink_ready,
	sink_sop,
	sink_eop,
	
	// streaming source
	source_data,
	source_valid,
	source_ready,
	source_sop,
	source_eop,
	
	// conduit
	mode
);

// global clock & reset
input	clk;
input	reset_n;

// mm slave
input							s_chipselect;
input							s_read;
input							s_write;
output	reg	[31:0]	s_readdata;
input	[31:0]				s_writedata;
input	[2:0]					s_address;


// streaming sink
input	[23:0]            	sink_data;
input								sink_valid;
output							sink_ready;
input								sink_sop;
input								sink_eop;

// streaming source
output	[23:0]			  	   source_data;
output								source_valid;
input									source_ready;
output								source_sop;
output								source_eop;

// conduit export
input                         mode;

////////////////////////////////////////////////////////////////////////
//
parameter IMAGE_W = 11'd640;
parameter IMAGE_H = 11'd480;
parameter MESSAGE_BUF_MAX = 256;
parameter MSG_INTERVAL = 6;
parameter BB_COL_DEFAULT = 24'h00ff00;


wire [7:0]   red, green, blue, grey;
wire [7:0]   red_out, green_out, blue_out;

wire         sop, eop, in_valid, out_ready;
////////////////////////////////////////////////////////////////////////
///Declaring variables for HSV conversion
wire[7:0] hue ;
wire[7:0] saturation, value, min;


// Detect red areas
//wire red_detect;
//assign red_detect = red[7] & ~green[7] & ~blue[7];
// Find boundary of cursor box

///Conversion from RGB to HSV
assign value = (red > green) ? ((red > blue) ? red[7:0] : blue[7:0]) : (green > blue) ? green[7:0] : blue[7:0];						
assign min = (red < green)? ((red<blue) ? red[7:0] : blue[7:0]) : (green < blue) ? green [7:0] : blue[7:0];
assign saturation = (value - min)* 255 / value;
assign hue = (red == green && red == blue) ? 0 :((value != red)? (value != green) ? (((240*((value - min))+ (60* (red - green)))/(value-min))>>1):
                ((120*(value-min)+60*(blue - red))/(value - min)>>1): 
                (blue < green) ? ((60*(green - blue)/(value - min))>>1): (((360*(value-min) +(60*(green - blue)))/(value - min))>>1));				 

reg prev_detect_high_r, prev_high_r, prev_high_r2;
reg prev_detect_high_y, prev_high_y, prev_high_y2;
reg prev_detect_high_b, prev_high_b, prev_high_b2;
//HSV boundary values for each LED beacon colour
wire red_beacon_detect, white_beacon_detect, orange_beacon_detect, blue_beacon_detect;	
assign red_beacon_detect =  (((hue >= 0 && hue <= 6)) && ((value >  105 && saturation > 102) || (saturation > 82 && value > 168)))||(hue >=0 && hue <= 3 && saturation >= 216 && saturation <= 255 && value >= 51 && value <=98); //sat > 102 /* (hue <= 10 && saturation >= 200 && value >= 0) | (hue == 180 && saturation >= 200 && value >= 60 && value <= 75); */
assign orange_beacon_detect= (hue >= 10 && hue <=50 && saturation >= 181 && ((value >= 190) | (value >= 80 && value <= 120))) | (hue >= 10 && hue <=50 && saturation >= 220 && ((value >= 190) | (value >= 121 && value <= 170)));
assign blue_beacon_detect = (hue >= 100 && hue != 180 && hue < 200 && saturation > 175 && value > 49);

initial begin
	prev_detect_high_r <= 0;
	prev_high_r <= 0;
	prev_high_r2 <= 0;
	prev_detect_high_y <= 0;
	prev_high_y <= 0;
	prev_high_y2 <= 0;
	prev_detect_high_b <= 0;
	prev_high_b <= 0;
	prev_high_b2 <= 0;
end
//Eliminates impulse disturbances in pixels 
always@(negedge clk) begin
	prev_high_r2 = prev_high_r;
	prev_high_r = prev_detect_high_r;
	prev_detect_high_r = (red_beacon_detect);
	prev_high_y2 = prev_high_y;
	prev_high_y = prev_detect_high_y;
	prev_detect_high_y = (orange_beacon_detect);
	prev_high_b2 = prev_high_b;
	prev_high_b = prev_detect_high_b;
	prev_detect_high_b = (blue_beacon_detect);
	
end



// Highlights detected areas
wire [23:0] color_high;
assign grey = green[7:1] + red[7:2] + blue[7:2]; //Grey = green/2 + red/4 + blue/4
assign color_high  =  (red_beacon_detect && prev_detect_high_r && prev_high_r && prev_high_r2) ? {8'hff,8'h10,8'h0}  
	: ((orange_beacon_detect && prev_detect_high_y & prev_high_y2)? {8'hf0,8'hf0,8'h00}
	: ((blue_beacon_detect && prev_detect_high_b && prev_high_b2) ? {8'h00,8'h00,8'hff}
	: {grey,grey,grey}))  ;

// Show bounding box
wire [23:0] detection_box;
wire detection_box_active;
assign detection_box_active = (x == IMAGE_W/3) | (x == IMAGE_W - IMAGE_W/3); //Object detection range in the camera's vision
assign detection_box = detection_box_active ? {24'hffffff} : color_high; 

wire [23:0] new_image_r;
wire bb_active_r;
// If object is within the beacon HSV boundary values defined and within the detection box, the object is processed as detected
assign bb_active_r = (x == left_r && left_r > IMAGE_W/3 && left_r < IMAGE_W - IMAGE_W/3) | (x == right_r && right_r < IMAGE_W - IMAGE_W/3 && right_r > IMAGE_W/3) 
| (y == top_r && left_r > IMAGE_W/3 && left_y < IMAGE_W - IMAGE_W/3 && right_r < IMAGE_W - IMAGE_W/3 && right_r > IMAGE_W/3) | (y == bottom_r && left_r > IMAGE_W/3 && left_r < IMAGE_W - IMAGE_W/3 && right_r < IMAGE_W - IMAGE_W/3 && right_r > IMAGE_W/3);  
// If conditions for bb_active_r are met then bounding box around the object otherwise nothing happens
assign new_image_r = bb_active_r ? {24'hff1000} : detection_box;

wire [23:0] new_image_y;
wire bb_active_y;
assign bb_active_y = (x == left_y && left_y > IMAGE_W/3 && left_y < IMAGE_W - IMAGE_W/3) | (x == right_y && right_y < IMAGE_W - IMAGE_W/3 && right_y > IMAGE_W/3) 
| (y == top_y && left_y > IMAGE_W/3 && left_y < IMAGE_W - IMAGE_W/3 && right_y < IMAGE_W - IMAGE_W/3 && right_y > IMAGE_W/3) | (y == bottom_y && left_y > IMAGE_W/3 && left_y < IMAGE_W - IMAGE_W/3 && right_y < IMAGE_W - IMAGE_W/3 && right_y > IMAGE_W/3); 
assign new_image_y = bb_active_y ? {24'hf0f000} : new_image_r;

wire [23:0] new_image_b;
wire bb_active_b;
assign bb_active_b = (x == left_b && left_b > IMAGE_W/3 && left_b < IMAGE_W - IMAGE_W/3) | (x == right_b && right_b < IMAGE_W - IMAGE_W/3 && right_b > IMAGE_W/3) 
| (y == top_b && left_b > IMAGE_W/3 && left_b < IMAGE_W - IMAGE_W/3 && right_b < IMAGE_W - IMAGE_W/3 && right_b > IMAGE_W/3) | (y == bottom_b && left_b > IMAGE_W/3 && left_b < IMAGE_W - IMAGE_W/3 && right_b < IMAGE_W - IMAGE_W/3 && right_b > IMAGE_W/3);  
assign new_image_b = bb_active_b ? {24'h0000ff} : new_image_y;


// Switch output pixels depending on mode switch
// Don't modify the start-of-packet word - it's a packet discriptor
// Don't modify data in non-video packets
assign {red_out, green_out, blue_out} = (mode & ~sop & packet_video) ? new_image_b : {red,green,blue};

//Count valid pixels to tget the image coordinates. Reset and detect packet type on Start of Packet.
reg [15:0] x, y;
reg packet_video;
always@(posedge clk) begin
	if (sop) begin
		x <= 11'h0;
		y <= 11'h0;
		packet_video <= (blue[3:0] == 3'h0);
	end
	else if (in_valid) begin
		if (x == IMAGE_W-1) begin
			x <= 11'h0;
			y <= y + 11'h1;
		end
		else begin
			x <= x + 11'h1;
		end
	end
end

//Find first and last red pixels
reg [15:0] x_min_r, x_max_r, y_min_r, y_max_r, x_min_y, x_max_y, y_min_y, y_max_y, x_min_w, x_max_w, y_min_w, y_max_w, x_min_b, x_max_b, y_min_b, y_max_b;
wire [10:0] y_dist_r, y_dist_y, y_dist_b;

// Pixel height of bounding box
assign y_dist_r = (y_min_r > y_max_r) ? 0 : (y_max_r-y_min_r);
assign y_dist_y = (y_min_y > y_max_y) ? 0 : (y_max_y-y_min_y);
assign y_dist_b = (y_min_b > y_max_b) ? 0 : (y_max_b-y_min_b);





initial begin
	x_min_r <= 0;
	x_max_r <= 0;
	x_min_y <= 0;
	x_max_y <= 0;
//	x_min_w <= 0;
//	x_max_w <= 0;
	x_min_b <= 0;
	x_max_b <= 0;
	y_min_r <= 0;
	y_max_r <= 0;
	y_min_y <= 0;
	y_max_y <= 0;
//	y_min_w <= 0;
//	y_max_w <= 0;
	y_min_b <= 0;
	y_max_b <= 0;
end

always@(posedge clk) begin
	//if (((green_beacon_detect || blue_beacon_detect || red_beacon_detect || grey_beacon_detect) && prev_detect_high && prev_high) & in_valid) begin	//Update bounds when the pixel is red
	if ((red_beacon_detect && prev_detect_high_r && prev_high_r && prev_high_r2 && new_image_r != detection_box) & in_valid) begin
		if (x < x_min_r) x_min_r <= x;
		if (x > x_max_r) x_max_r <= x;
      if (y < y_min_r) y_min_r <= y;
		y_max_r <= y;
	end
	if ((orange_beacon_detect && prev_detect_high_y && prev_high_y && prev_high_y2 && new_image_y != new_image_r) & in_valid) begin
		if (x < x_min_y) x_min_y <= x;
		if (x > x_max_y) x_max_y <= x;
      if (y < y_min_y) y_min_y <= y;
		y_max_y <= y;
	end
	if ((blue_beacon_detect && prev_detect_high_b && prev_high_b && prev_high_b2 && new_image_b != new_image_y) & in_valid) begin
		if (x < x_min_b) x_min_b <= x;
		if (x > x_max_b) x_max_b <= x;
      if (y < y_min_b) y_min_b <= y;
		y_max_b <= y;
	end
	if (sop & in_valid) begin	//Reset bounds on start of packet
        x_min_r <= IMAGE_W-11'h1;
        x_max_r <= 0;
        x_min_y <= IMAGE_W-11'h1;
        x_max_y <= 0;
        x_min_b <= IMAGE_W-11'h1;
        x_max_b <= 0;

        y_min_r <= IMAGE_H-11'h1;
        y_max_r <= 0;
        y_min_y <= IMAGE_H-11'h1;
        y_max_y <= 0;
        y_min_b <= IMAGE_H-11'h1;
        y_max_b <= 0;
    end
//	 if (zoom_state == 1 && (y_dist_r > 50 || y_dist_y > 50 || y_dist_b > 50)) begin
//		zoom_bit = (new_image_b != new_image_y) | (new_image_y != new_image_r) | (new_image_r != detection_box) ? 8'b1 : 8'b0;
//	 end

end


//Process bounding box at the end of the frame.
reg [1:0] msg_state;
reg [15:0] left_r, right_r, top_r, bottom_r, left_y, right_y, top_y, bottom_y, left_b, right_b, top_b, bottom_b;
reg [7:0] frame_count;
//wire [7:0] zoom_state;
wire [7:0] stop_bit;
//assign zoom_state = (new_image_r != detection_box && y_dist_r > 0 && y_dist_r < 46) | (new_image_y != new_image_r && y_dist_y > 0 && y_dist_y < 46) | (new_image_b != new_image_y && y_dist_b > 0 && y_dist_b < 46) ? 8'b1 : 8'b0;
assign stop_bit =  (new_image_b != new_image_y) | (new_image_y != new_image_r) | (new_image_r != detection_box) ? 8'b1 : 8'b0;
//always@(*) begin
//	if (y_dist_r > 0 && y_dist_r < 46 && zoom_state == 0) begin
//		zoom_state <= 16'b1;
//	end
//	else begin
//		zoom_state <= 0;
//	end
//	if (y_dist_y > 0 && y_dist_y < 46 && zoom_state == 0) begin
//		zoom_state <= 16'b1;
//	end
//	else begin 
//		zoom_state <= 0;
//	end
//	if (y_dist_b > 0 && y_dist_b < 46 && zoom_state == 0) begin
//		zoom_state <= 16'b1;
//	end
//	else begin
//		zoom_state <= 0;
//	end
//end
//Process bounding box at the end of the frame.
always@(posedge clk) begin
    if (eop & in_valid & packet_video) begin  //Ignore non-video packets
    
        //Latch edges for display overlay on next frame
        left_r <= x_min_r;
        right_r <= x_max_r;
        top_r <= y_min_r;
        bottom_r <= y_max_r;  
        left_y <= x_min_y;
        right_y <= x_max_y;
        top_y <= y_min_y;
        bottom_y <= y_max_y;  
        left_b <= x_min_b;
        right_b <= x_max_b;
        top_b <= y_min_b;
        bottom_b <= y_max_b;
		
		//Start message writer FSM once every MSG_INTERVAL frames, if there is room in the FIFO
		frame_count <= frame_count - 1;
		
		if (frame_count == 0 && msg_buf_size < MESSAGE_BUF_MAX - 3) begin
			msg_state <= 2'b01;
			frame_count <= MSG_INTERVAL-1;
		end
	end
	
	//Cycle through message writer states once started
	if (msg_state != 2'b00) msg_state <= msg_state + 2'b01;

end

//Generate output messages for CPU
reg [31:0] msg_buf_in; 
wire [31:0] msg_buf_out;
reg msg_buf_wr;
wire msg_buf_rd, msg_buf_flush;
wire [7:0] msg_buf_size;
wire msg_buf_empty;
`define RED_BOX_MSG_ID "RBB"
wire[6:0] r1,r2;
assign r1 = 16'd79;
assign r2 = 16'd20;

wire [12:0] constan;
assign constan = 16'd7443;
reg [31:0] distance_r1, distance_y1, distance_b1;

always @(posedge clk)begin
	if(y_min_r != IMAGE_W-11'h1 && y_max_r != 0) begin 
		distance_r1 = (y_dist_r < 97) ? ((constan * r1)/r2/y_dist_r) / 10 : ((((constan - (((y_dist_r - 97) * 5)/16))* r1)/r2)/y_dist_r) / 10;
	end
	else begin
		distance_r1 = 0;
	end
	if (y_min_y != IMAGE_W-11'h1 && y_max_y != 0) begin
		distance_y1 = (y_dist_y < 97) ? ((constan * r1)/r2/y_dist_y) / 10 : ((((constan - (((y_dist_y - 97) * 5)/16))* r1)/r2)/y_dist_y) / 10;
	end
	else begin
		distance_y1 = 0;
	end
	if (y_min_b != IMAGE_W-11'h1 && y_max_b != 0) begin
		distance_b1 = (y_dist_b < 97) ? ((constan * r1)/r2/y_dist_b) / 10: ((((constan - (((y_dist_b - 97) * 5)/16))* r1)/r2)/y_dist_b) / 10;
	end
	else begin
		distance_b1 = 0;
	end
end


reg [10:0] outt_r, outt_y, outt_b;
//reg [7:0] outt_zoom_state;
reg [7:0] outt_stop_bit; 
//79/20 = 3.95 -> 3

always@(*) begin	//Write words to FIFO as state machine advances
	case(msg_state)
		2'b00: begin
			msg_buf_in = 32'd0; 
			msg_buf_wr = 1'b0;
		end
		2'b01: begin
			outt_r = distance_r1[15:0];
//			outt_zoom_state = zoom_state[7:0];
			outt_stop_bit = stop_bit[7:0];
			msg_buf_in = {`RED_BOX_MSG_ID};	//Message ID 
			msg_buf_wr = 1'b1;
		end
		2'b10: begin
//			outt_r = y_dist_r[10:0];
			outt_y = distance_y1[15:0];
			outt_b = distance_b1[15:0];
			msg_buf_in = {8'b0, outt_stop_bit, distance_r1}; // red and yellow beacon bounding box pixel heights
			msg_buf_wr = 1'b1; 
		end
		2'b11: begin
			//msg_buf_in = {5'b0, x_max, 5'b0, y_max};	//Top left coordinate
			msg_buf_in = {distance_y1, distance_b1}; //blue bounding box pixel height
			msg_buf_wr = 1'b1;  
		end
	endcase
end


//Output message FIFO
MSG_FIFO	MSG_FIFO_inst (
	.clock (clk),
	.data (msg_buf_in),
	.rdreq (msg_buf_rd),
	.sclr (~reset_n | msg_buf_flush),
	.wrreq (msg_buf_wr),
	.q (msg_buf_out),
	.usedw (msg_buf_size),
	.empty (msg_buf_empty)
	);


//Streaming registers to buffer video signal
STREAM_REG #(.DATA_WIDTH(26)) in_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(sink_ready),
	.valid_out(in_valid),
	.data_out({red,green,blue,sop,eop}),
	.ready_in(out_ready),
	.valid_in(sink_valid),
	.data_in({sink_data,sink_sop,sink_eop})
);

STREAM_REG #(.DATA_WIDTH(26)) out_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(out_ready),
	.valid_out(source_valid),
	.data_out({source_data,source_sop,source_eop}),
	.ready_in(source_ready),
	.valid_in(in_valid),
	.data_in({red_out, green_out, blue_out, sop, eop})
);


/////////////////////////////////
/// Memory-mapped port		 /////
/////////////////////////////////

// Addresses
`define REG_STATUS    			0
`define READ_MSG    				1
`define READ_ID    				2
`define REG_BBCOL					3

//Status register bits
// 31:16 - unimplemented
// 15:8 - number of words in message buffer (read only)
// 7:5 - unused
// 4 - flush message buffer (write only - read as 0)
// 3:0 - unused


// Process write

reg  [7:0]   reg_status;
reg	[23:0]	bb_col;

always @ (posedge clk)
begin
	if (~reset_n)
	begin
		reg_status <= 8'b0;
		bb_col <= BB_COL_DEFAULT;
	end
	else begin
		if(s_chipselect & s_write) begin
		   if      (s_address == `REG_STATUS)	reg_status <= s_writedata[7:0];
		   if      (s_address == `REG_BBCOL)	bb_col <= s_writedata[23:0];
		end
	end
end


//Flush the message buffer if 1 is written to status register bit 4
assign msg_buf_flush = (s_chipselect & s_write & (s_address == `REG_STATUS) & s_writedata[4]);


// Process reads
reg read_d; //Store the read signal for correct updating of the message buffer

// Copy the requested word to the output port when there is a read.
always @ (posedge clk)
begin
   if (~reset_n) begin
	   s_readdata <= {32'b0};
		read_d <= 1'b0;
	end
	
	else if (s_chipselect & s_read) begin
		if   (s_address == `REG_STATUS) s_readdata <= {16'b0,msg_buf_size,reg_status};
		if   (s_address == `READ_MSG) s_readdata <= {msg_buf_out};
		if   (s_address == `READ_ID) s_readdata <= 32'h1234EEE2;
		if   (s_address == `REG_BBCOL) s_readdata <= {8'h0, bb_col};
	end
	
	read_d <= s_read;
end

//Fetch next word from message buffer after read from READ_MSG
assign msg_buf_rd = s_chipselect & s_read & ~read_d & ~msg_buf_empty & (s_address == `READ_MSG);
						


endmodule

