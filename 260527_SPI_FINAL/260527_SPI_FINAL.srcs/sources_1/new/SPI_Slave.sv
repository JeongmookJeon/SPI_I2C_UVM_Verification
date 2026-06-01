`timescale 1ns / 1ps

module spi_slave (
    input  logic       clk,
    input  logic       rst,
    input  logic       sclk,
    input  logic       cs_n,
    input  logic       mosi,
    input  logic [7:0] tx_data,
    output logic [7:0] rx_data,
    output logic       miso,
    output logic       done,
    output logic       busy
);

    typedef enum logic [2:0] {
        IDLE    = 3'd0,
        START   = 3'd1,
        DATA_RX = 3'd2,
        DATA_TX = 3'd3,
        STOP    = 3'd4
    } state_e;

    state_e c_state, n_state;

    logic [7:0] rx_data_next;
    logic [7:0] tx_shift_reg, tx_shift_next;
    logic [7:0] rx_shift_reg, rx_shift_next;
    logic [3:0] bit_cnt_reg, bit_cnt_next;
    logic miso_next;
    logic done_next, busy_next;

    logic sclk_pedge, sclk_nedge;

    edge_detector U_EDGE_DETECTOR (
        .clk    (clk),
        .rst    (rst),
        .data_in(sclk),
        .pedge  (sclk_pedge),
        .nedge  (sclk_nedge)
    );

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            c_state      <= IDLE;
            rx_data      <= 8'd0;
            tx_shift_reg <= 8'd0;
            rx_shift_reg <= 8'd0;
            miso         <= 1'b0;
            done         <= 1'b0;
            busy         <= 1'b0;
            bit_cnt_reg  <= 4'd0;
        end else begin
            c_state      <= n_state;
            rx_data      <= rx_data_next;
            tx_shift_reg <= tx_shift_next;
            rx_shift_reg <= rx_shift_next;
            miso         <= miso_next;
            done         <= done_next;
            busy         <= busy_next;
            bit_cnt_reg  <= bit_cnt_next;
        end
    end

    always_comb begin
        n_state       = c_state;
        rx_data_next  = rx_data;
        tx_shift_next = tx_shift_reg;
        rx_shift_next = rx_shift_reg;
        miso_next     = miso;
        done_next     = 1'b0;  // done은 1클럭 펄스
        busy_next     = busy;
        bit_cnt_next  = bit_cnt_reg;

        case (c_state)
            IDLE: begin
                miso_next = 1'b0;
                busy_next = 1'b0;
                if (!cs_n) begin
                    tx_shift_next = tx_data;  // 전송할 데이터 로드
                    rx_shift_next = 8'd0;
                    bit_cnt_next  = 4'd0;
                    busy_next     = 1'b1;
                    n_state       = START;
                end
            end
            // TX & RX 동시 발생.
            // 첫번째 BIT 교환
            START: begin
                miso_next = tx_shift_reg[7];  // MSB 먼저 출력
                if (sclk_pedge) begin
                    rx_shift_next = {rx_shift_reg[6:0], mosi};
                    tx_shift_next = {tx_shift_reg[6:0], 1'b0};
                    bit_cnt_next  = 4'd1;
                    n_state       = DATA_TX;
                end
            end
            // Slave -> Master
            DATA_TX: begin
                if (sclk_nedge) begin
                    miso_next = tx_shift_reg[7];
                    n_state   = DATA_RX;
                end
                // cs_n이 해제
                if (cs_n) begin
                    n_state = IDLE;
                end
            end
            DATA_RX: begin
                if (sclk_pedge) begin
                    rx_shift_next = {rx_shift_reg[6:0], mosi};
                    tx_shift_next = {tx_shift_reg[6:0], 1'b0};
                    bit_cnt_next  = bit_cnt_reg + 4'd1;
                    if (bit_cnt_reg == 4'd7) begin
                        n_state = STOP;
                    end else begin
                        n_state = DATA_TX;
                    end
                end
                if (cs_n) begin
                    n_state = IDLE;
                end
            end
            STOP: begin
                rx_data_next = rx_shift_reg;
                done_next    = 1'b1;
                busy_next    = 1'b0;
                miso_next    = 1'b0;
                n_state      = IDLE;
            end
            default: n_state = IDLE;
        endcase
    end

endmodule



// Edge Detector: SCLK 에지 검출
module edge_detector (
    input  logic clk,
    input  logic rst,
    input  logic data_in,
    output logic pedge,
    output logic nedge
);
    logic ff;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) ff <= 1'b0;
        else ff <= data_in;
    end

    assign pedge = ~ff & data_in;  // 0→1
    assign nedge = ff & ~data_in;  // 1→0
endmodule
