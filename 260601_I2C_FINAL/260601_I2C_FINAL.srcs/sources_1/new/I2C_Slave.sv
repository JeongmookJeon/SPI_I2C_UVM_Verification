`timescale 1ns / 1ps

module I2C_SLAVE #(
    parameter logic [6:0] SLA_ADDR = 7'h12
) (
    input logic clk,
    input logic reset,

    input logic [7:0] tx_data,
    output logic [7:0] rx_data,
    output logic done,
    output logic busy,

    input logic scl,
    inout wire  sda
);

    logic sda_o, sda_i;

    assign sda_i = sda;
    assign sda   = sda_o ? 1'bz : 1'b0;

    i2c_slave #(
        .SLA_ADDR(SLA_ADDR)
    ) u_i2c_slave (
        .*,
        .sda_o(sda_o),
        .sda_i(sda_i)
    );

endmodule

module i2c_slave #(
    parameter logic [6:0] SLA_ADDR = 7'h12
) (
    input logic clk,
    input logic reset,

    input logic [7:0] tx_data,
    output logic [7:0] rx_data,
    output logic done,
    output logic busy,

    input  logic scl,
    output logic sda_o,
    input  logic sda_i
);

    typedef enum logic [2:0] {
        IDLE     = 3'b000,
        ADDR     = 3'b001,
        ACK_ADDR = 3'b010,
        DATA     = 3'b011,
        DATA_ACK = 3'b100
    } i2c_state_e;

    i2c_state_e state;

    logic [2:0] scl_sync, sda_sync;
    logic sda_r;
    logic [7:0] tx_shift_reg, rx_shift_reg;
    logic [3:0] bit_cnt;
    logic is_read;

    assign sda_o = sda_r;
    assign busy  = (state != IDLE);

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            scl_sync <= 3'b111;
            sda_sync <= 3'b111;
        end else begin
            scl_sync <= {scl_sync[1:0], scl};
            sda_sync <= {sda_sync[1:0], sda_i};
        end
    end

    wire scl_high = (scl_sync[1] == 1'b1);
    wire scl_rise = (scl_sync[2:1] == 2'b01);
    wire scl_fall = (scl_sync[2:1] == 2'b10);

    wire sda_fall = (sda_sync[2:1] == 2'b10);
    wire sda_rise = (sda_sync[2:1] == 2'b01);

    wire start_cond = scl_high && sda_fall;
    wire stop_cond = scl_high && sda_rise;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state        <= IDLE;
            sda_r        <= 1'b1;
            done         <= 1'b0;
            tx_shift_reg <= 0;
            rx_shift_reg <= 0;
            is_read      <= 1'b0;
            bit_cnt      <= 0;
            rx_data      <= 0;

        end else if (start_cond) begin
            state   <= ADDR;
            bit_cnt <= 0;
            sda_r   <= 1'b1;
            done    <= 1'b0;

        end else if (stop_cond) begin
            state <= IDLE;
            sda_r <= 1'b1;

        end else begin
            done <= 1'b0;

            case (state)
                IDLE: begin
                    sda_r <= 1'b1;
                end

                ADDR: begin
                    if (scl_rise) begin
                        rx_shift_reg <= {rx_shift_reg[6:0], sda_sync[1]};
                    end
                    if (scl_fall) begin
                        // 7을 8로 바꿈. 
                        if (bit_cnt == 8) begin
                            if (rx_shift_reg[7:1] == SLA_ADDR) begin
                                state   <= ACK_ADDR;
                                // 0: Write, 1: Read
                                is_read <= rx_shift_reg[0]; // 0번째 bit를 보고 read / write판단. 
                                sda_r   <= 1'b0;
                            end else begin
                                state <= IDLE;
                            end
                            bit_cnt <= 0;
                        end else begin
                            bit_cnt <= bit_cnt + 1;
                        end
                    end
                end

                ACK_ADDR: begin
                    if (scl_fall) begin
                        state   <= DATA;
                        bit_cnt <= 0;
                        if (is_read) begin
                            tx_shift_reg <= tx_data;
                            sda_r        <= tx_data[7];
                        end else begin
                            sda_r <= 1'b1;
                        end
                    end
                end

                DATA: begin
                    if (scl_rise) begin
                        if (!is_read) begin
                            rx_shift_reg <= {rx_shift_reg[6:0], sda_sync[1]};
                        end
                    end
                    if (scl_fall) begin
                        if (bit_cnt == 7) begin
                            state   <= DATA_ACK;
                            bit_cnt <= 0;
                            done    <= 1'b1;

                            if (!is_read) begin
                                rx_data <= rx_shift_reg;
                                sda_r   <= 1'b0;
                            end else begin
                                sda_r <= 1'b1;
                            end
                        end else begin
                            bit_cnt <= bit_cnt + 1;
                            if (is_read) begin
                                tx_shift_reg <= {tx_shift_reg[6:0], 1'b0};
                                sda_r        <= tx_shift_reg[6];
                            end
                        end
                    end
                end

                DATA_ACK: begin
                    if (scl_rise) begin
                        if (is_read && sda_sync[1] == 1'b1) begin
                            state <= IDLE;
                        end
                    end
                    if (scl_fall) begin
                        state <= DATA;
                        if (is_read) begin
                            tx_shift_reg <= tx_data;
                            sda_r        <= tx_data[7];
                        end else begin
                            sda_r <= 1'b1;
                        end
                    end
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule
