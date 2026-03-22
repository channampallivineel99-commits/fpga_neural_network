module conv_256_16 (
	parameter IMG_WIDTH       = 256,
	parameter IMG_HEIGHT      = 256,
	parameter PIXEL_WIDTH     = 8,
	parameter WR_DATA_WIDTH   = 32,
	parameter RD_DATA_WIDTH   = 64,
	parameter BLOCK_SIZE      = 16,
	parameter THRESHOLD       = 32768    // Accumulator threshold
)(
	input wire clk,
	input wire rst_n,

	// Internal AXI-Stream Interface (from skid buffer)
	input wire [WR_DATA_WIDTH-1:0] int_tdata,
	input wire int_tvalid,
	input wire int_tlast,
	output wire int_tready,

	// Convolution Output
	output reg [255:0] data_out,
	output reg data_out_valid,
	input wire data_out_ready,

	input wire send_new_img,

	// Status
	input wire [15:0] accumulator_threshold,
	output wire [15:0] wr_cnt_out,
	output wire [1:0] busy
);

	//===========================================
	// Derived Parameters
	//===========================================
	localparam IMG_SIZE        = IMG_WIDTH * IMG_HEIGHT;             // 65536
	localparam PIXELS_PER_WR   = WR_DATA_WIDTH / PIXEL_WIDTH;        // 4
	localparam PIXELS_PER_RD   = RD_DATA_WIDTH / PIXEL_WIDTH;        // 8
	localparam WR_DEPTH        = IMG_SIZE / PIXELS_PER_WR;           // 16384
	localparam RD_DEPTH        = IMG_SIZE / PIXELS_PER_RD;           // 8192
	localparam WR_ADDR_WIDTH   = $clog2(WR_DEPTH);                   // 14
	localparam RD_ADDR_WIDTH   = $clog2(RD_DEPTH);                   // 13
	localparam WR_BE_WIDTH     = WR_DATA_WIDTH / 8;                  // 4
	localparam BLOCKS_H        = IMG_WIDTH / BLOCK_SIZE;             // 16
	localparam BLOCKS_V        = IMG_HEIGHT / BLOCK_SIZE;            // 16
	localparam TOTAL_BLOCKS    = BLOCKS_H * BLOCKS_V;                // 256
	localparam BLOCK_PIXELS    = BLOCK_SIZE * BLOCK_SIZE;            // 256
	localparam ACC_WIDTH       = PIXEL_WIDTH + $clog2(BLOCK_PIXELS); // 16

	// Reads per block row: BLOCK_SIZE pixels / PIXELS_PER_RD = 16/8 = 2
	localparam RD_PER_BLK_ROW  = BLOCK_SIZE / PIXELS_PER_RD;         // 2
	localparam RD_LATENCY      = 2;

	//===========================================
	// Write Port Signals
	//===========================================
	reg bram_wr_en;
	reg [WR_ADDR_WIDTH-1:0] bram_wr_addr;
	reg [WR_DATA_WIDTH-1:0] bram_wr_data;
	wire [WR_BE_WIDTH-1:0] bram_wr_be;

	assign bram_wr_be = {WR_BE_WIDTH{1'b1}};

	//============================================================
	// Read Port Signals
	//============================================================
	reg                             bram_rd_en;
	reg [RD_ADDR_WIDTH-1:0]         bram_rd_addr;
	wire [RD_DATA_WIDTH-1:0]        bram_rd_data;

	//============================================================
	// State Machine Parameters
	//============================================================
	// Write FSM States
	localparam WR_IDLE     = 2'd0;
	localparam WR_WRITE    = 2'd1;
	localparam WR_WAIT     = 2'd2;
	localparam WR_DONE     = 2'd3;

	// Read FSM States
	localparam RD_IDLE         = 3'd0;
	localparam RD_SETUP        = 3'd1;
	localparam RD_READ         = 3'd2;
	localparam RD_ACCUMULATE   = 3'd3;
	localparam RD_THRESHOLD    = 3'd4;
	localparam RD_NEXT_BLOCK   = 3'd5;
	localparam RD_OUTPUT       = 3'd6;
	localparam RD_DONE         = 3'd7;

	//============================================================
	// Write FSM Signals
	//============================================================
	reg [1:0]                   wr_state;
	reg [WR_ADDR_WIDTH-1:0]     wr_addr_cnt;         // Counter for write address
	reg                         write_done;          // Flag: write phase complete
	reg                         wr_sm_busy;          // Write FSM busy indicator

	//============================================================
	// Read FSM Signals
	//============================================================
	reg [2:0]                   rd_state;
	reg                         read_done;           // Flag: read phase complete
	reg                         output_consumed;     // Flag: output was accepted
	reg                         rd_sm_busy;          // Read FSM busy indicator

	//============================================================
	// Block Tracking Registers
	// Tracks current position in 16x16 block grid
	//============================================================
	reg [3:0]                   blk_row;        // Current block row (0..15)
	reg [3:0]                   blk_col;        // Current block column (0..15)
	reg [7:0]                   blk_index;      // Linear block index (0..255)

	//============================================================
	// Intra-Block Tracking Registers
	// Tracks position within current 16x16 block
	//============================================================
	reg [3:0]                   local_row;      // Row within block (0..15)
	reg [{$clog2(RD_PER_BLK_ROW)-1:0}] local_rd;  // Read within block row (0..1)

	//============================================================
	// Read Pipeline Signals
	// Tracks BRAM read latency (2 cycles)
	//============================================================
	reg [1:0]                   rd_latency_cnt; // Latency counter for BRAM reads
	reg                         rd_data_valid_pipe [0:RD_LATENCY-1];

	//============================================================
	// Accumulator Registers
	// Accumulates pixel values across entire 16x16 block
	//============================================================
	reg [ACC_WIDTH-1:0]         accumulator;
	reg [{$clog2(BLOCK_PIXELS)-1:0}] block_acc_cnt;  // Count of accumulated pixels in block

	//============================================================
	// Result Register
	// Stores final 256-bit output (one bit per block)
	//============================================================
	reg [255:0]                 result_reg;

	//============================================================
	// Output Port Assignments
	//============================================================
	// TREADY: accept data only during write phase
	assign int_tready  = (wr_state == WR_WRITE);
	// busy[1:0] = {wr_sm_busy, rd_sm_busy}
	assign busy        = {wr_sm_busy, rd_sm_busy};
	// Output write counter for debugging/monitoring
	assign wr_cnt_out  = wr_addr_cnt;

	//=============================================================
	// Write State Machine
	// Purpose: Accept AXI-Stream input data and write to BRAM
	// States: IDLE -> WRITE -> WAIT -> DONE -> (IDLE)
	//=============================================================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			wr_state     <= WR_IDLE;
			wr_addr_cnt  <= {WR_ADDR_WIDTH{1'b0}};
			bram_wr_en   <= 1'b0;
			bram_wr_addr <= {WR_ADDR_WIDTH{1'b0}};
			bram_wr_data <= {WR_DATA_WIDTH{1'b0}};
			write_done   <= 1'b0;
			wr_sm_busy   <= 1'b0;
		end else begin
			bram_wr_en <= 1'b0; // Default: no write

			case (wr_state)
				//=================================================
				// WR_IDLE: Wait for signal to start new image write
				//=================================================
				WR_IDLE: begin
					wr_addr_cnt <= {WR_ADDR_WIDTH{1'b0}};
					write_done  <= 1'b0;
					wr_state    <= WR_WRITE;
				end

				//=================================================
				// WR_WRITE: Accept AXI-Stream beats, write to BRAM
				// Writes 4 pixels per cycle (32-bit data)
				//=================================================
				WR_WRITE: begin
					if (int_tvalid && int_tready) begin
						// Write accepted: write to BRAM
						bram_wr_en   <= 1'b1;
						bram_wr_addr <= wr_addr_cnt;
						bram_wr_data <= int_tdata;

						if (wr_addr_cnt == WR_DEPTH - 1) begin
							// All data written (16384 words = 65536 pixels)
							write_done <= 1'b1;
							wr_state   <= send_new_img ? WR_IDLE : WR_WAIT;
						end else begin
							// Increment address for next write
							wr_addr_cnt <= wr_addr_cnt + 1'b1;
						end
						wr_sm_busy <= 1'b1;
					end else begin
						// No valid data: stay in write or wait for new image
						wr_state <= send_new_img ? WR_IDLE : WR_WRITE;
					end
				end

				//=================================================
				// WR_WAIT: Wait for read FSM to complete processing
				// before allowing new image to be written
				//=================================================
				WR_WAIT: begin
					if (read_done) begin
						wr_state   <= send_new_img ? WR_IDLE : WR_DONE;
						wr_sm_busy <= 1'b0;
					end
				end

				//=================================================
				// WR_DONE: Wait for output to be consumed
				// Prevents overwriting before previous result is read
				//=================================================
				WR_DONE: begin
					if (output_consumed) begin
						write_done <= 1'b0;
						wr_state   <= WR_IDLE;
					end
				end

				default: wr_state <= WR_IDLE;
			endcase
		end
	end

	//============================================================
	// Read Address Calculation
	// Computes BRAM read address based on block position
	// and intra-block row/read index
	//============================================================
	// Formula:
	//   Pixel row    = blk_row * 16 + local_row
	//   Pixel col    = blk_col * 16 + local_rd * 8
	//   Linear pixel = pixel_row * 256 + pixel_col
	//   RD address   = linear_pixel / 8 (since 64-bit read = 8 pixels)
	//============================================================
	reg [RD_ADDR_WIDTH-1:0] rd_addr_calc;

	always @(*) begin
		rd_addr_calc = ((blk_row << 4) + local_row) * (IMG_WIDTH / PIXELS_PER_RD)
		             + (blk_col << 1) + local_rd;
	end

	//============================================================
	// Pixel Extraction from 64-bit Read Data
	// Extracts 8 individual 8-bit pixels from BRAM read output
	// 64-bit word contains: [7:0], [15:8], [23:16], ..., [63:56]
	//============================================================
	wire [PIXEL_WIDTH-1:0] rd_pixels [0:PIXELS_PER_RD-1];
	
	genvar g;
	generate
		for (g = 0; g < PIXELS_PER_RD; g = g + 1) begin : pixel_extract
			assign rd_pixels[g] = bram_rd_data[g*PIXEL_WIDTH +: PIXEL_WIDTH];
		end
	endgenerate

	//============================================================
	// Pixel Sum Calculation
	// Sums all 8 pixels from one 64-bit read
	// Result width accommodates sum without overflow
	//============================================================
	wire [ACC_WIDTH-1:0] pixel_sum;
	assign pixel_sum = {{(ACC_WIDTH-PIXEL_WIDTH){1'b0}}, rd_pixels[0]}
	                 + {{(ACC_WIDTH-PIXEL_WIDTH){1'b0}}, rd_pixels[1]}
	                 + {{(ACC_WIDTH-PIXEL_WIDTH){1'b0}}, rd_pixels[2]}
	                 + {{(ACC_WIDTH-PIXEL_WIDTH){1'b0}}, rd_pixels[3]}
	                 + {{(ACC_WIDTH-PIXEL_WIDTH){1'b0}}, rd_pixels[4]}
	                 + {{(ACC_WIDTH-PIXEL_WIDTH){1'b0}}, rd_pixels[5]}
	                 + {{(ACC_WIDTH-PIXEL_WIDTH){1'b0}}, rd_pixels[6]}
	                 + {{(ACC_WIDTH-PIXEL_WIDTH){1'b0}}, rd_pixels[7]};

	//=============================================================
	// Read Data Valid Pipeline
	// Tracks BRAM read latency (2 cycles)
	// Ensures data is valid when used in accumulation
	//=============================================================
	reg rd_request;
	reg [RD_LATENCY:0] rd_valid_sr;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rd_valid_sr <= {(RD_LATENCY+1){1'b0}};
		end else begin
			rd_valid_sr <= {rd_valid_sr[RD_LATENCY-1:0], rd_request};
		end
	end

	wire rd_data_valid = rd_valid_sr[RD_LATENCY];

	//=============================================================
	// Read State Machine
	// Purpose: Read blocks from BRAM, accumulate, threshold,
	//          and output 256-bit result (1 bit per block)
	// States: IDLE -> SETUP -> READ -> ACCUMULATE -> THRESHOLD
	//         -> NEXT_BLOCK -> OUTPUT -> DONE -> (IDLE)
	//=============================================================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rd_state        <= RD_IDLE;
			read_done       <= 1'b0;
			output_consumed <= 1'b0;
			blk_row         <= 4'd0;
			blk_col         <= 4'd0;
			blk_index       <= 8'd0;
			local_row       <= 4'd0;
			local_rd        <= 1'b0;
			accumulator     <= {ACC_WIDTH{1'b0}};
			result_reg      <= 256'd0;
			bram_rd_en      <= 1'b0;
			bram_rd_addr    <= {RD_ADDR_WIDTH{1'b0}};
			rd_request      <= 1'b0;
			data_out        <= 256'd0;
			data_out_valid  <= 1'b0;
			block_acc_cnt   <= 4'd0;
			rd_sm_busy      <= 1'b0;
		end else begin
			// Defaults: disable read, no request
			bram_rd_en <= 1'b0;
			rd_request <= 1'b0;

			//========================================================
			// Output Handshake Logic
			// Manages data_out_valid and output_consumed signals
			//========================================================
			if (data_out_valid && data_out_ready) begin
				// Output accepted by downstream: clear valid
				data_out_valid  <= 1'b0;
				output_consumed <= 1'b1;
			end else if (output_consumed) begin
				// Clear consumed flag after one cycle
				data_out_valid  <= 1'b0;
				output_consumed <= 1'b0;
			end

			case (rd_state)
				//=================================================
				// RD_IDLE: Wait for write phase completion
				// Blocks until all image data is written to BRAM
				//=================================================
				RD_IDLE: begin
					read_done     <= 1'b0;
					blk_row       <= 4'd0;
					blk_col       <= 4'd0;
					blk_index     <= 8'd0;
					local_row     <= 4'd0;
					local_rd      <= 1'b0;
					accumulator   <= {ACC_WIDTH{1'b0}};
					block_acc_cnt <= 0;
					if (write_done && !data_out_valid) begin
						// Ready to process: move to setup
						rd_state   <= RD_SETUP;
						rd_sm_busy <= 1'b1;
					end
				end

				//=================================================
				// RD_SETUP: Initialize block processing
				// Reset accumulators and counters for new block
				//=================================================
				RD_SETUP: begin
					accumulator   <= {ACC_WIDTH{1'b0}};
					local_row     <= 4'd0;
					local_rd      <= 1'b0;
					block_acc_cnt <= 0;
					rd_state      <= send_new_img ? RD_IDLE : RD_READ;
				end

				//=================================================
				// RD_READ: Issue BRAM read request
				// Reads 8 pixels (64-bit) from current block position
				//=================================================
				RD_READ: begin
				    bram_rd_en   <= 1'b1;
				    bram_rd_addr <= rd_addr_calc;
				    rd_request   <= 1'b1;
				
				    if (local_rd == RD_PER_BLK_ROW - 1) begin
				        local_rd <= 1'b0;
				        // Advance to next row within block
				        if (local_row == BLOCK_SIZE - 1) begin
				            local_row <= 0;
				            rd_state  <= send_new_img ? RD_IDLE : RD_ACCUMULATE;
				        end else begin
				            local_row <= local_row + 1'b1;
				        end
				    end else begin
				        local_rd <= local_rd + 1'b1;
				    end
				
				    if (rd_data_valid) begin
				        accumulator   <= accumulator + pixel_sum;
				        block_acc_cnt <= block_acc_cnt + 1;
				    end
				end

				//=================================================
				// RD_ACCUMULATE: Wait for read data and accumulate
				// Sums pixel values until all 256 pixels processed
				// (32 reads x 8 pixels = 256 pixels)
				//=================================================
				RD_ACCUMULATE: begin
					if (rd_data_valid) begin
						// Data from BRAM is ready: add to accumulator
						accumulator   <= accumulator + pixel_sum;
						block_acc_cnt <= block_acc_cnt + 1;
						
						// Check if all pixels in block accumulated
						// (block has 256 pixels / 8 pixels per read = 32 reads = 0x1F+1)
						if (block_acc_cnt >= 'h1F) begin
							rd_state <= send_new_img ? RD_IDLE : RD_THRESHOLD;
						end else begin
							// Need more reads: request next read
							rd_state <= RD_READ;
						end
					end
				end

				//=================================================
				// RD_THRESHOLD: Compare accumulator to threshold
				// Sets output bit: 1 if below threshold, 0 if above
				//=================================================
				RD_THRESHOLD: begin
					if (accumulator >= accumulator_threshold) begin
						// Accumulator exceeds threshold: output 0
						result_reg[blk_index] <= 1'b0;
					end else begin
						// Accumulator below threshold: output 1
						result_reg[blk_index] <= 1'b1;
					end
					rd_state <= send_new_img ? RD_IDLE : RD_NEXT_BLOCK;
				end

				//=================================================
				// RD_NEXT_BLOCK: Advance to next block
				// Updates block coordinates or signals completion
				//=================================================
				RD_NEXT_BLOCK: begin
					if (blk_index == TOTAL_BLOCKS - 1) begin
						// All 256 blocks processed: output result
						rd_state <= send_new_img ? RD_IDLE : RD_OUTPUT;
					end else begin
						// Move to next block
						blk_index <= blk_index + 1'b1;
							
						// Update 2D block coordinates
						if (blk_col == BLOCKS_H - 1) begin
							// End of row: wrap column, increment row
							blk_col <= 4'd0;
							blk_row <= blk_row + 1'b1;
						end else begin
							// Continue across row
							blk_col <= blk_col + 1'b1;
						end
							rd_state <= send_new_img ? RD_IDLE : RD_SETUP;
					end
				end

				//=================================================
				// RD_OUTPUT: Present final result on output port
				// Holds result until downstream accepts (data_out_ready)
				//=================================================
				RD_OUTPUT: begin
					data_out       <= result_reg;
					data_out_valid <= 1'b1;
					read_done      <= 1'b1;
					rd_state       <= send_new_img ? RD_IDLE : RD_DONE;
					rd_sm_busy     <= 1'b0;
				end

				//=================================================
				// RD_DONE: Wait for output consumption
				// Ensures result is read before returning to idle
				//=================================================
				RD_DONE: begin
					if (output_consumed) begin
						rd_state <= send_new_img ? RD_IDLE : RD_DONE;;
					end
				end

				default: rd_state <= RD_IDLE;
			endcase
		end
	end

	//=============================================================
	// XPM Simple Dual Port RAM (SDPRAM)
	// Configuration:
	//   - Memory size: 65536 pixels x 8 bits = 524288 bits
	//   - Write port (A): 32-bit data (4 pixels), 14-bit address
	//   - Read port (B):  64-bit data (8 pixels), 13-bit address
	//   - Read latency:   2 cycles
	//   - Primitive:      Block RAM
	//=============================================================
	xpm_memory_sdpram #(
		.MEMORY_SIZE        (IMG_SIZE * PIXEL_WIDTH),    // 524288 bits
		.MEMORY_PRIMITIVE   ("block"),
		.CLOCKING_MODE      ("common_clock"),
		.ECC_MODE           ("no_ecc"),
		.MEMORY_INIT_FILE   ("none"),
		.MEMORY_INIT_PARAM  ("0"),
		.USE_MEM_INIT       (1),
		.USE_MEM_INIT_MMI   (0),
		.WAKEUP_TIME        ("disable_sleep"),
		.AUTO_SLEEP_TIME    (0),
		.MESSAGE_CONTROL    (0),
		.USE_EMBEDDED_CONSTRAINT (0),
		.MEMORY_OPTIMIZATION ("true"),
		.CASCADE_HEIGHT     (0),
		.SIM_ASSERT_CHK     (1),
		.WRITE_PROTECT      (1),

		// Write Port A Configuration
		.WRITE_DATA_WIDTH_A (WR_DATA_WIDTH),             // 32 bits (4 pixels)
		.BYTE_WRITE_WIDTH_A (PIXEL_WIDTH),               // 8 bits (1 pixel)
		.ADDR_WIDTH_A       (WR_ADDR_WIDTH),             // 14 bits
		.RST_MODE_A         ("SYNC"),

		// Read Port B Configuration
		.READ_DATA_WIDTH_B  (RD_DATA_WIDTH),             // 64 bits (8 pixels)
		.ADDR_WIDTH_B       (RD_ADDR_WIDTH),             // 13 bits
		.READ_RESET_VALUE_B ("0"),
		.READ_LATENCY_B     (RD_LATENCY),                // 2 cycles
		.WRITE_MODE_B       ("no_change"),
		.RST_MODE_B         ("SYNC")
	) u_sdpram (
		// Write Port A (input side)
		.clka               (clk),
		.ena                (bram_wr_en),
		.wea                (bram_wr_be),
		.addra              (bram_wr_addr),
		.dina               (bram_wr_data),
		.injectsbiterra     (1'b0),
		.injectdbiterra     (1'b0),
		.sleep              (1'b0),

		// Read Port B (output side)
		.clkb               (clk),
		.enb                (bram_rd_en),
		.rstb               (~rst_n),
		.regceb             (1'b1),
		.addrb              (bram_rd_addr),
		.doutb              (bram_rd_data),
		.sbiterrb           (),
		.dbiterrb           ()
	);

endmodule
