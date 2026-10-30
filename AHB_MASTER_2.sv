module AHB_MASTER_2 #(
    parameter ADDR_BITS = 32,
    parameter DATA_BITS = 32
)(
    input  logic                 hclk, 
    input  logic                 hreset, // Active-low reset (HRESETn)

    // Signal from client/user who drives the master
    input  logic                 write,
    input  logic                 cmd_valid,
    input  logic                 cmd_last, // Denotes end of INCR burst
    input  logic [1:0]           size,
    input  logic [ADDR_BITS-1:0] addr_in,
    input  logic [DATA_BITS-1:0] wt_data,
    input  logic [2:0]           burst,
    output logic                 cmd_ready,
    output logic [DATA_BITS-1:0] rd_data,
    output logic                 rd_valid,
    output logic                 error,

    // Master to Slave interface
    input  logic                 hready,
    input  logic                 hresp,
    input  logic [DATA_BITS-1:0] hrdata,
    output logic                 hwrite,
    output logic                 hreadyout,
    output logic [ADDR_BITS-1:0] haddr,
    output logic [DATA_BITS-1:0] hwdata,
    output logic [2:0]           hburst,
    output logic [1:0]           htrans,
    output logic [2:0]           hsize
);

    // AHB HTRANS states
    localparam IDLE   = 2'd0;
    localparam NONSEQ = 2'd2;
    localparam SEQ    = 2'd3;

    // FSM State and beat counter
    logic [1:0] master_stage;
    logic [3:0] cnt;

    // Data phase tracking registers
    logic write_phase;
    logic read_phase;

    // Direct interface assignments
    assign error     = hresp;
    assign hreadyout = hready;
    assign hwdata    = wt_data; // Driven directly during write data phase

    // Address & Burst sizing calculations
    logic [ADDR_BITS-1:0] burst_len;
    logic [ADDR_BITS-1:0] data_len;
    logic [ADDR_BITS-1:0] wrap_size;
    logic [ADDR_BITS-1:0] wrap_base;

    always_comb begin
        case (hburst)
            3'd2, 3'd3: burst_len = ADDR_BITS'(4);
            3'd4, 3'd5: burst_len = ADDR_BITS'(8);
            3'd6, 3'd7: burst_len = ADDR_BITS'(16);
            default:    burst_len = ADDR_BITS'(1);
        endcase
        data_len  = ADDR_BITS'($unsigned(1 << hsize));
        wrap_size = burst_len * data_len;
        wrap_base = haddr & ~(wrap_size - 1'b1);
    end

    // Function to calculate next sequential or wrapped address
    function automatic logic [ADDR_BITS-1:0] get_next_addr(
        input logic [ADDR_BITS-1:0] current_addr,
        input logic [2:0]           burst_type,
        input logic [ADDR_BITS-1:0] d_len,
        input logic [ADDR_BITS-1:0] w_base,
        input logic [ADDR_BITS-1:0] w_size
    );
        logic [ADDR_BITS-1:0] nxt;
        nxt = current_addr + d_len;
        if (burst_type == 3'd2 || burst_type == 3'd4 || burst_type == 3'd6) begin // WRAP
            if (nxt >= (w_base + w_size))
                return w_base;
            else
                return nxt;
        end else begin // INCR
            return nxt;
        end
    endfunction

    // Command Ready handshake logic
    always_comb begin
        cmd_ready = 1'b0;
        case (master_stage)
            IDLE: begin
                cmd_ready = 1'b1;
            end
            NONSEQ: begin
                if (hready && (hburst == 3'd0 || (hburst == 3'd1 && cmd_last))) begin
                    cmd_ready = 1'b1;
                end
            end
            SEQ: begin
                if (hready) begin
                    if (hburst == 3'd1) begin
                        if (cmd_last) cmd_ready = 1'b1;
                    end else if (cnt == 4'd1) begin
                        cmd_ready = 1'b1;
                    end
                end
            end
            default: cmd_ready = 1'b0;
        endcase
    end

    // Data Phase Tracking & Read Sampling Logic
    always_ff @(posedge hclk or negedge hreset) begin
        if (!hreset) begin
            write_phase <= 1'b0;
            read_phase  <= 1'b0;
            rd_data     <= '0;
            rd_valid    <= 1'b0;
        end else begin
            rd_valid <= 1'b0; // Default pulse high for 1 cycle

            if (hready) begin
                write_phase <= ((htrans == NONSEQ) || (htrans == SEQ)) && hwrite;
                read_phase  <= ((htrans == NONSEQ) || (htrans == SEQ)) && !hwrite;

                if (read_phase) begin
                    rd_data  <= hrdata;
                    rd_valid <= 1'b1;
                end
            end
        end
    end

    // Master FSM & Control Output Logic
    always_ff @(posedge hclk or negedge hreset) begin
        if (!hreset) begin
            master_stage <= IDLE;
            htrans       <= IDLE;
            haddr        <= '0;
            hwrite       <= 1'b0;
            hsize        <= '0;
            hburst       <= '0;
            cnt          <= 4'd0;
        end else begin
            case (master_stage)

                IDLE: begin
                    if (cmd_valid && hready) begin
                        haddr        <= addr_in;
                        hsize        <= {1'b0, size};
                        hwrite       <= write;
                        hburst       <= burst;
                        htrans       <= NONSEQ;
                        master_stage <= NONSEQ;
                        case (burst)
                            3'd2, 3'd3: cnt <= 4'd3;  // 4-beat burst
                            3'd4, 3'd5: cnt <= 4'd7;  // 8-beat burst
                            3'd6, 3'd7: cnt <= 4'd15; // 16-beat burst
                            default:    cnt <= 4'd0;  // SINGLE / INCR
                        endcase
                    end else begin
                        htrans       <= IDLE;
                        master_stage <= IDLE;
                    end
                end

                NONSEQ: begin
                    if (hready) begin
                        if (hburst == 3'd0) begin // SINGLE transfer
                            if (cmd_valid) begin
                                haddr        <= addr_in;
                                hsize        <= {1'b0, size};
                                hwrite       <= write;
                                hburst       <= burst;
                                htrans       <= NONSEQ;
                                master_stage <= NONSEQ;
                                case (burst)
                                    3'd2, 3'd3: cnt <= 4'd3;
                                    3'd4, 3'd5: cnt <= 4'd7;
                                    3'd6, 3'd7: cnt <= 4'd15;
                                    default:    cnt <= 4'd0;
                                endcase
                            end else begin
                                htrans       <= IDLE;
                                master_stage <= IDLE;
                            end
                        end else if (hburst == 3'd1) begin // INCR transfer
                            if (cmd_last) begin
                                if (cmd_valid) begin
                                    haddr        <= addr_in;
                                    hsize        <= {1'b0, size};
                                    hwrite       <= write;
                                    hburst       <= burst;
                                    htrans       <= NONSEQ;
                                    master_stage <= NONSEQ;
                                    case (burst)
                                        3'd2, 3'd3: cnt <= 4'd3;
                                        3'd4, 3'd5: cnt <= 4'd7;
                                        3'd6, 3'd7: cnt <= 4'd15;
                                        default:    cnt <= 4'd0;
                                    endcase
                                end else begin
                                    htrans       <= IDLE;
                                    master_stage <= IDLE;
                                end
                            end else begin
                                haddr        <= haddr + data_len;
                                htrans       <= SEQ;
                                master_stage <= SEQ;
                            end
                        end else begin // Fixed length burst (WRAP / INCR 4,8,16)
                            haddr        <= get_next_addr(haddr, hburst, data_len, wrap_base, wrap_size);
                            htrans       <= SEQ;
                            master_stage <= SEQ;
                        end
                    end
                end

                SEQ: begin
                    if (hready) begin
                        if (hburst == 3'd1) begin // Unspecified INCR
                            if (cmd_last) begin
                                if (cmd_valid) begin
                                    haddr        <= addr_in;
                                    hsize        <= {1'b0, size};
                                    hwrite       <= write;
                                    hburst       <= burst;
                                    htrans       <= NONSEQ;
                                    master_stage <= NONSEQ;
                                    case (burst)
                                        3'd2, 3'd3: cnt <= 4'd3;
                                        3'd4, 3'd5: cnt <= 4'd7;
                                        3'd6, 3'd7: cnt <= 4'd15;
                                        default:    cnt <= 4'd0;
                                    endcase
                                end else begin
                                    htrans       <= IDLE;
                                    master_stage <= IDLE;
                                end
                            end else begin
                                haddr        <= haddr + data_len;
                                htrans       <= SEQ;
                                master_stage <= SEQ;
                            end
                        end else begin // Fixed length burst
                            if (cnt == 4'd1) begin // Last address beat of burst
                                if (cmd_valid) begin
                                    haddr        <= addr_in;
                                    hsize        <= {1'b0, size};
                                    hwrite       <= write;
                                    hburst       <= burst;
                                    htrans       <= NONSEQ;
                                    master_stage <= NONSEQ;
                                    case (burst)
                                        3'd2, 3'd3: cnt <= 4'd3;
                                        3'd4, 3'd5: cnt <= 4'd7;
                                        3'd6, 3'd7: cnt <= 4'd15;
                                        default:    cnt <= 4'd0;
                                    endcase
                                end else begin
                                    htrans       <= IDLE;
                                    master_stage <= IDLE;
                                end
                            end else begin
                                cnt          <= cnt - 1'b1;
                                haddr        <= get_next_addr(haddr, hburst, data_len, wrap_base, wrap_size);
                                htrans       <= SEQ;
                                master_stage <= SEQ;
                            end
                        end
                    end
                end

                default: begin
                    master_stage <= IDLE;
                    htrans       <= IDLE;
                end
            endcase
        end
    end

endmodule