// Simple AXI4-Lite Slave Example
// Two registers: CONTROL (writeable) and STATUS (read-only)

module axi4lite_slave #
(
    parameter ADDR_WIDTH = 4,
    parameter DATA_WIDTH = 32
)
(
    input  wire                  ACLK,
    input  wire                  ARESETN,

    // Write Address Channel
    input  wire [ADDR_WIDTH-1:0] AWADDR,
    input  wire                  AWVALID,
    output reg                   AWREADY,

    // Write Data Channel
    input  wire [DATA_WIDTH-1:0] WDATA,
    input  wire [(DATA_WIDTH/8)-1:0] WSTRB,
    input  wire                  WVALID,
    output reg                   WREADY,

    // Write Response Channel
    output reg [1:0]             BRESP,
    output reg                   BVALID,
    input  wire                  BREADY,

    // Read Address Channel
    input  wire [ADDR_WIDTH-1:0] ARADDR,
    input  wire                  ARVALID,
    output reg                   ARREADY,

    // Read Data Channel
    output reg [DATA_WIDTH-1:0]  RDATA,
    output reg [1:0]             RRESP,
    output reg                   RVALID,
    input  wire                  RREADY

    output  wire [DATA_WIDTH-1:0]  control_out,
    input   wire [DATA_WIDTH-1:0]  status_in
);

    // Internal registers
    reg [DATA_WIDTH-1:0] control_reg;
    reg [DATA_WIDTH-1:0] status_reg;

    // Example status register increments every cycle
    assign control_out = control_reg;

    always @(posedge ACLK) begin
        if (!ARESETN)
            status_reg <= 0;
        else
            status_reg <= status_in;
    end

    // Write logic
    always @(posedge ACLK) begin
        if (!ARESETN) begin
            AWREADY <= 0;
            WREADY  <= 0;
            BVALID  <= 0;
            BRESP   <= 2'b00;
            control_reg <= 0;
        end else begin
            // Accept write address
            if (AWVALID && !AWREADY)
                AWREADY <= 1;
            else
                AWREADY <= 0;

            // Accept write data
            if (WVALID && !WREADY) begin
                WREADY <= 1;
                if (AWADDR == 4'h0) begin
                    // Write to control register
                    if (WSTRB[0]) control_reg[7:0]   <= WDATA[7:0];
                    if (WSTRB[1]) control_reg[15:8]  <= WDATA[15:8];
                    if (WSTRB[2]) control_reg[23:16] <= WDATA[23:16];
                    if (WSTRB[3]) control_reg[31:24] <= WDATA[31:24];
                end
            end else
                WREADY <= 0;

            // Write response
            if (WVALID && AWVALID && !BVALID) begin
                BVALID <= 1;
                BRESP  <= 2'b00; // OKAY
            end else if (BVALID && BREADY) begin
                BVALID <= 0;
            end
        end
    end

    // Read logic
    always @(posedge ACLK) begin
        if (!ARESETN) begin
            ARREADY <= 0;
            RVALID  <= 0;
            RRESP   <= 2'b00;
            RDATA   <= 0;
        end else begin
            if (ARVALID && !ARREADY)
                ARREADY <= 1;
            else
                ARREADY <= 0;

            if (ARVALID && !RVALID) begin
                RVALID <= 1;
                RRESP  <= 2'b00; // OKAY
                case (ARADDR)
                    4'h0: RDATA <= control_reg;
                    4'h4: RDATA <= status_reg;
                    default: RDATA <= 32'hDEADBEEF; // invalid address
                endcase
            end else if (RVALID && RREADY) begin
                RVALID <= 0;
            end
        end
    end

endmodule
