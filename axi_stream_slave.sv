// Simple AXI-Stream Slave with Memory Buffer
// Accepts AXI-Stream input and writes data to internal FIFO memory

module axis_slave_fifo #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH = 16,  // Memory buffer depth
    parameter ADDR_WIDTH = $clog2(DEPTH)
)(
    input  wire                  ACLK,
    input  wire                  ARESETN,

    // AXI-Stream Slave Interface
    input  wire [DATA_WIDTH-1:0] S_AXIS_TDATA,
    input  wire                  S_AXIS_TVALID,
    output reg                   S_AXIS_TREADY,
    input  wire                  S_AXIS_TLAST,

    // Status signals
    output reg                   fifo_full,
    output reg                   fifo_empty,
    output reg [ADDR_WIDTH:0]    fifo_count  // Count of items in FIFO
);

    // Memory buffer
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] wr_ptr;  // Write pointer
    reg [ADDR_WIDTH-1:0] rd_ptr;  // Read pointer
    reg [ADDR_WIDTH:0]   count;   // Number of stored items

    // Combinational logic for FIFO status
    always @(*) begin
        fifo_full  = (count == DEPTH);
        fifo_empty = (count == 0);
        fifo_count = count;
        S_AXIS_TREADY = !fifo_full;  // Ready when not full
    end

    // Sequential logic for FIFO write and pointer management
    always @(posedge ACLK or negedge ARESETN) begin
        if (!ARESETN) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count  <= 0;
        end
        else begin
            // Write data to FIFO when slave is ready and valid data arrives
            if (S_AXIS_TVALID && S_AXIS_TREADY) begin
                mem[wr_ptr] <= S_AXIS_TDATA;
                wr_ptr <= wr_ptr + 1;
                count  <= count + 1;
            end

            // Optionally read data from FIFO (for testing/verification)
            // This would be uncommented if you want to push data out
        end
    end

    // Generate a pulse when transaction completes (optional)
    reg last_transaction;
    always @(posedge ACLK or negedge ARESETN) begin
        if (!ARESETN)
            last_transaction <= 1'b0;
        else
            last_transaction <= (S_AXIS_TVALID && S_AXIS_TREADY && S_AXIS_TLAST);
    end

endmodule
