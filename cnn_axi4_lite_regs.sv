// Register Map:
// 0x00 - REG_THRESHOLD       (R/W) : [15:0]  accumulator_threshold
// 0x04 - REG_STATUS          (R)   : [0]     conv_busy
// 0x08 - REG_CONTROL         (R/W) : [0]     soft_reset (self-clearing 8 cyc)
// 0x0C - REG_VERSION         (R)   : [31:0]  design_version
// 0x10 - REG_WEIGHT_DATA     (R/W) : [FP_TW_WB-1:0] mem_weight_wdata
// 0x14 - REG_WEIGHT_ADDR     (R/W) : [7:0]   mem_wr_addr
// 0x18 - REG_WEIGHT_WEN      (R/W) : [0]     mem_wr_en (self-clearing 1 cyc)
// 0x1C - REG_BIAS_DATA       (R/W) : [0]     bias_in
// 0x20 - REG_BIAS_VLD        (R/W) : [0]     bias_vld (self-clearing 1 cyc)
// 0x24 - REG_C_SEL           (R/W) : [clog2(NODES_PER_HL*NUM_HL + 10)-1:0] c_sel,

//=============================================================================
// AXI4-Lite Slave Register Module
//=============================================================================

module axi_lite_regs #(
    parameter AXI_ADDR_WIDTH    = 8,
    parameter AXI_DATA_WIDTH    = 32,
    parameter DEFAULT_THRESHOLD = 16'd32768,
    parameter DESIGN_VERSION    = 32'h0001_0000,
    parameter FP_TW_WB          = 8,
    parameter NODES_PER_HL      = 64,
    parameter NUM_HL            = 1
)(
    input wire                  aclk,
    input wire                  areset_n,

    input wire                  clk,
    input wire                  rst,

    //========================================================================
    // AXI4-Lite Slave Interface
    //========================================================================

    // Write Address Channel
    input wire [AXI_ADDR_WIDTH-1:0]    s_axi_awaddr,
    input wire [2:0]                   s_axi_awprot,
    input wire                         s_axi_awvalid,
    output reg                         s_axi_awready,

    // Write Data Channel
    input wire [AXI_DATA_WIDTH-1:0]    s_axi_wdata,
    input wire [AXI_DATA_WIDTH/8-1:0]  s_axi_wstrb,
    input wire                         s_axi_wvalid,
    output reg                         s_axi_wready,

    // Write Response Channel
    output reg [1:0]                   s_axi_bresp,
    output reg                         s_axi_bvalid,
    input wire                         s_axi_bready,

    // Read Address Channel
    input wire [AXI_ADDR_WIDTH-1:0]    s_axi_araddr,
    input wire [2:0]                   s_axi_arprot,
    input wire                         s_axi_arvalid,
    output reg                         s_axi_arready,

    // Read Data Channel
    output reg [AXI_DATA_WIDTH-1:0]    s_axi_rdata,
    output reg [1:0]                   s_axi_rresp,
    output reg                         s_axi_rvalid,
    input wire                         s_axi_rready,

    //========================================================================
    // Convolution Control/Status
    //========================================================================
    output reg [15:0]                  accumulator_threshold,
    input wire [1:0]                   soft_reset,
    input wire [15:0]                  conv_busy,
    output wire [1:0]                  write_count,
    output wire [15:0]                 new_image_send,

    //========================================================================
    // Neural Network Configuration
    //========================================================================
    output reg [FP_TW_WB-1:0]          mem_weight_wdat,
    output reg [$clog2(256)-1:0]       mem_wr_addr,
    output reg                         mem_wr_en,
    output reg [FP_TW_WB-1:0]          bias_in,
    output reg                         bias_vld,
    output reg [$clog2(NODES_PER_HL*NUM_HL + 10)-1:0] c_sel,

    input wire [9:0]                   classify_out,
    output wire                        classify_out_valid,
    input wire                         classify_out_ready
);

    localparam STR8_WIDTH    = AXI_DATA_WIDTH / 8;
    localparam WR_ADDR_WIDTH = $clog2(NODES_PER_HL * NUM_HL + 10);
    localparam C_SEL_WIDTH   = $clog2(NODES_PER_HL * NUM_HL + 10);

    //========================================================================
    // Register Address Offsets (byte-addressed, word-aligned)
    //========================================================================
    localparam ADDR_THRESHOLD      = 8'h00;
    localparam ADDR_STATUS         = 8'h04;
    localparam ADDR_CONTROL        = 8'h08;
    localparam ADDR_VERSION        = 8'h0C;
    localparam ADDR_WEIGHT_DATA    = 8'h10;
    localparam ADDR_WEIGHT_ADDR    = 8'h14;
    localparam ADDR_WEIGHT_WEN     = 8'h18;
    localparam ADDR_BIAS_DATA      = 8'h1C;
    localparam ADDR_BIAS_VLD       = 8'h20;
    localparam ADDR_C_SEL          = 8'h24;
    localparam ADDR_CLASSIFY_STATUS = 8'h28;
    localparam ADDR_CLASSIFY_RESULT = 8'h2C;

    // AXI Response Codes
    localparam RESP_OKAY   = 2'b00;
    localparam RESP_DECERR = 2'b11;

    //========================================================================
    // Internal signals
    //========================================================================
    reg [AXI_ADDR_WIDTH-1:0]            wr_addr_latched;
    reg [AXI_ADDR_WIDTH-1:0]            rd_addr_latched;
    reg                                 aw_latched;
    reg                                 w_latched;

    // Self-clear counters
    reg [3:0]                           soft_reset_cnt;
    reg                                 mem_wr_en_pending;
    reg                                 bias_vld_pending;

    wire [10:0]                         src_classify_data;
    wire [10:0]                         dst_classify_data;

    reg [4:0]                           aclk_cdata_rdy;
    reg                                 rdy_pulse_to_cdc;

    reg [3:0]                           new_img_send_pipe;
    reg                                 new_img_send_src_p;

    reg [3:0]                           rd_status_latch_pipe;
    reg                                 rd_status_latch_src_p;
    reg                                 rd_status_latch;

    wire [1:0]                          conv_busy_user;
    wire [15:0]                         write_count_user;
    wire [15:0]                         write_count_int;

    assign src_classify_data = {classify_out_valid, classify_out};
    assign dst_classify_data = {(classify_out_valid | classify_out_ready), classify_out};

    reg                                 s_axi_arready_pipe;

    assign write_count = write_count_user;

    always @(posedge aclk or negedge areset_n) begin
        if (!areset_n) begin
            s_axi_awready   <= 1'b0;
            aw_latched      <= 1'b0;
            wr_addr_latched <= {AXI_ADDR_WIDTH{1'b0}};
        end else begin
            s_axi_awready <= !aw_latched;
            if (s_axi_awvalid && s_axi_awready) begin
                aw_latched      <= 1'b1;
                wr_addr_latched <= s_axi_awaddr;
            end else if (s_axi_bvalid && s_axi_bready) begin
                aw_latched <= 1'b0;
            end
        end
    end

    always @(posedge aclk or negedge areset_n) begin
        if (!areset_n) begin
            s_axi_wready <= 1'b0;
            w_latched    <= 1'b0;
        end else if (!w_latched && s_axi_wvalid) begin
            s_axi_wready <= 1'b1;
            w_latched    <= 1'b1;
        end else if (w_latched && s_axi_wvalid) begin
            s_axi_wready <= 1'b0;
        end
    end

    //========================================================================
    // Write Logic: Register Updates
    //========================================================================
    always @(posedge aclk or negedge areset_n) begin
        if (!areset_n) begin
            accumulator_threshold   <= DEFAULT_THRESHOLD;
            soft_reset              <= 1'b0;
            soft_reset_cnt          <= 4'd0;
            mem_weight_wdat         <= {FP_TW_WB{1'b0}};
            mem_wr_addr             <= {WR_ADDR_WIDTH{1'b0}};
            mem_wr_en               <= 1'b0;
            mem_wr_en_pending       <= 1'b0;
            bias_in                 <= {FP_TW_WB{1'b0}};
            bias_vld                <= 1'b0;
            bias_vld_pending        <= 1'b0;
            c_sel                   <= {$clog2(NODES_PER_HL*NUM_HL + 10){1'b0}};
            s_axi_bvalid            <= 1'b0;
            s_axi_bresp             <= RESP_OKAY;
        end else begin
            s_axi_bvalid <= 1'b0; // Default

            // Self-clearing: soft_reset (8 cycles)
            if (soft_reset_cnt > 0) begin
                soft_reset_cnt <= soft_reset_cnt - 1'b1;
                if (soft_reset_cnt == 4'd1) begin
                    soft_reset <= 1'b0;
                end
            end

            // Self-clearing: mem_wr_en (1 cycle)
            if (mem_wr_en && !mem_wr_en_pending) begin
                mem_wr_en <= 1'b0;
            end
            if (mem_wr_en_pending) begin
                mem_wr_en           <= 1'b0;
                mem_wr_en_pending   <= 1'b0;
            end

            // Self-clearing: bias_vld (1 cycle)
            if (bias_vld && !bias_vld_pending) begin
                bias_vld <= 1'b0;
            end
            if (bias_vld_pending) begin
                bias_vld            <= 1'b0;
                bias_vld_pending    <= 1'b0;
            end

            // Write Response Handshake
            if (aw_latched && w_latched && !s_axi_bvalid) begin
                s_axi_bvalid <= 1'b1;
            end

            // Perform Write When Both Address And Data Are Latched
            if (aw_latched && w_latched && !s_axi_bvalid) begin
                case (wr_addr_latched[7:0])
                    //-------- 0x00 - REG_THRESHOLD: Accumulator threshold [15:0]
                    ADDR_THRESHOLD: begin
                        accumulator_threshold <= s_axi_wdata[15:0];
                    end

                    //-------- 0x04 - REG_STATUS: Read-only, writes ignored
                    ADDR_STATUS: begin
                        s_axi_bresp <= RESP_OKAY;
                    end

                    //-------- 0x08 - REG_CONTROL: Bit[0] soft_reset (self-clearing)
                    ADDR_CONTROL: begin
                        if (s_axi_wdata[0]) begin
                            soft_reset      <= 1'b1;
                            soft_reset_cnt  <= 4'd8;
                        end
                    end

                    //-------- 0x0C - REG_VERSION: Read-only, writes ignored
                    ADDR_VERSION: begin
                        s_axi_bresp <= RESP_OKAY;
                    end

                    //-------- 0x10 - REG_WEIGHT_DATA: Weight data [FP_TW_WB-1:0]
                    ADDR_WEIGHT_DATA: begin
                        mem_weight_wdat <= s_axi_wdata[FP_TW_WB-1:0];
                    end

                    //-------- 0x14 - REG_WEIGHT_ADDR: Weight address [7:0]
                    ADDR_WEIGHT_ADDR: begin
                        mem_wr_addr <= s_axi_wdata[WR_ADDR_WIDTH-1:0];
                    end

                    //-------- 0x18 - REG_WEIGHT_WEN: Bit[0] mem_wr_en (self-clearing 1 cyc)
                    ADDR_WEIGHT_WEN: begin
                        if (s_axi_wdata[0]) begin
                            mem_wr_en           <= 1'b1;
                            mem_wr_en_pending   <= 1'b1;
                        end
                    end

                    //-------- 0x1C - REG_BIAS_DATA: Bias data [FP_TW_WB-1:0]
                    ADDR_BIAS_DATA: begin
                        bias_in <= s_axi_wdata[FP_TW_WB-1:0];
                    end

                    //-------- 0x20 - REG_BIAS_VLD: Bias valid (self-clearing after 1 cycle)
                    ADDR_BIAS_VLD: begin
                        if (s_axi_wdata[0]) begin
                            bias_vld_pending <= 1'b1;
                            bias_vld         <= 1'b1;
                        end
                    end

                    //-------- 0x24 - REG_C_SEL: Node/layer select
                    ADDR_C_SEL: begin
                        c_sel <= s_axi_wdata[C_SEL_WIDTH-1:0];
                    end

                    //-------- 0x28 - REG_CLASSIFY_STATUS: Read-only, writes ignored
                    ADDR_CLASSIFY_STATUS: begin
                        s_axi_bresp <= RESP_OKAY;
                    end

                    //-------- 0x2C - REG_CLASSIFY_RESULT: Read-only, writes ignored
                    ADDR_CLASSIFY_RESULT: begin
                        s_axi_bresp <= RESP_OKAY;
                    end

                    //-------- Unmapped address: decode error
                    default: begin
                        s_axi_bresp <= RESP_DECERR;
                    end
                endcase
            end
        end
    end

    //========================================================================
    // Address/Data Latching and Response Generation
    //========================================================================
    always @(posedge aclk or negedge areset_n) begin
        if (!areset_n) begin
            s_axi_arready  <= 1'b0;
            rd_addr_latched <= {AXI_ADDR_WIDTH{1'b0}};
        end else begin
            if (s_axi_arvalid && s_axi_arready) begin
                s_axi_arready  <= 1'b0;
                rd_addr_latched <= s_axi_araddr;
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_arready  <= 1'b1;
            end
        end
    end

    //========================================================================
    // Read Data Channel Handling
    //========================================================================
    always @(posedge aclk or negedge areset_n) begin
        if (!areset_n) begin
            s_axi_rdata  <= {AXI_DATA_WIDTH{1'b0}};
            s_axi_rresp  <= RESP_OKAY;
            s_axi_rvalid <= 1'b0;
            rdy_pulse_to_cdc <= 0;
            aclk_cdata_rdy   <= 0;
        end else begin
            if (s_axi_arvalid && s_axi_arready) begin
                s_axi_rvalid <= 1'b1;
            end
            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end

            // Generate Read Data After Address Accepted ---
            if (s_axi_arready && s_axi_arvalid) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rresp  <= RESP_OKAY;
                s_axi_rdata  <= {AXI_DATA_WIDTH{1'b0}}; // Default zero-fill

                case (rd_addr_latched[7:0])
                    //-------- 0x00 - REG_THRESHOLD
                    ADDR_THRESHOLD: begin
                        s_axi_rdata <= {{(AXI_DATA_WIDTH-16){1'b0}}, accumulator_threshold};
                    end

                    //-------- 0x04 - REG_STATUS
                    //         Bit[1:0]: {conv_busy_write_sm, conv_busy_read_sm}
                    ADDR_STATUS: begin
                        s_axi_rdata <= {{(AXI_DATA_WIDTH-2){1'b0}}, conv_busy_user};
                    end

                    //-------- 0x08 - REG_CONTROL
                    //         Bit[0]: soft_reset (current state)
                    ADDR_CONTROL: begin
                        s_axi_rdata <= {{(AXI_DATA_WIDTH-1){1'b0}}, soft_reset};
                    end

                    //-------- 0x0C - REG_VERSION: Read-only, writes ignored
                    ADDR_VERSION: begin
                        s_axi_rdata <= DESIGN_VERSION;
                    end

                    //-------- 0x10 - REG_WEIGHT_DATA
                    ADDR_WEIGHT_DATA: begin
                        s_axi_rdata <= {{(AXI_DATA_WIDTH-FP_TW_WB){1'b0}}, mem_weight_wdat};
                    end

                    //-------- 0x14 - REG_WEIGHT_ADDR
                    ADDR_WEIGHT_ADDR: begin
                        s_axi_rdata <= {{(AXI_DATA_WIDTH-WR_ADDR_WIDTH){1'b0}}, mem_wr_addr};
                    end

                    //-------- 0x18 - REG_WEIGHT_WEN
                    ADDR_WEIGHT_WEN: begin
                        s_axi_rdata <= {{(AXI_DATA_WIDTH-1){1'b0}}, mem_wr_en};
                    end

                    //-------- 0x1C - REG_BIAS_DATA
                    ADDR_BIAS_DATA: begin
                        s_axi_rdata <= {{(AXI_DATA_WIDTH-FP_TW_WB){1'b0}}, bias_in};
                    end

                    //-------- 0x20 - REG_BIAS_VLD
                    ADDR_BIAS_VLD: begin
                        s_axi_rdata <= {{(AXI_DATA_WIDTH-1){1'b0}}, bias_vld};
                    end

                    //-------- 0x24 - REG_C_SEL: Node/layer select
                    ADDR_C_SEL: begin
                        s_axi_rdata <= {{(AXI_DATA_WIDTH-$clog2(NODES_PER_HL*NUM_HL + 10)){1'b0}}, c_sel};
                    end

                    //-------- 0x28 - REG_CLASSIFY_STATUS
                    //         Bit[1:0]: {conv_busy_write_sm, conv_busy_read_sm}
                    ADDR_CLASSIFY_STATUS: begin
                        s_axi_rdata <= {{(AXI_DATA_WIDTH-2){1'b0}}, {write_count_int, conv_busy_user}};
                    end

                    //-------- 0x2C - REG_CLASSIFY_RESULT
                    ADDR_CLASSIFY_RESULT: begin
                        s_axi_rdata <= {{(AXI_DATA_WIDTH-22){1'b0}}, dst_classify_data[9:0]};
                        // Note: read_clear_pulse_src is generated in the
                        // destination domain logic block above when this
                        // address is read and result is valid
                        aclk_cdata_rdy[0] <= 1'b1;
                    end

                    //-------- Unmapped address: decode error, return zero
                    default: begin
                        s_axi_rdata <= {AXI_DATA_WIDTH{1'b0}};
                        s_axi_rresp <= RESP_DECERR;
                    end
                endcase
            end
        end
    end

endmodule
