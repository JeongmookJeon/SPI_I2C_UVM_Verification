`include "uvm_macros.svh"
import uvm_pkg::*;

`timescale 1ns / 1ps


interface spi_if (
    input bit clk,
    input bit rst
);
    logic [7:0] clk_div;
    logic [7:0] tx_data_m;
    logic [7:0] tx_data_s;
    logic       start;
    logic [7:0] rx_data_m;
    logic [7:0] rx_data_s;
    logic       done_m;
    logic       busy_m;
    logic       done_s;
    logic       busy_s;

    clocking drv_cb @(posedge clk);
        default input #1step output #0;
        output clk_div;
        output tx_data_m;
        output tx_data_s;
        output start;
        input  rx_data_m;
        input  rx_data_s;
        input  done_m;
        input  busy_m;
        input  done_s;
        input  busy_s;
    endclocking

    clocking mon_cb @(posedge clk);
        default input #1step;
        input clk_div;
        input tx_data_m;
        input tx_data_s;
        input start;
        input rx_data_m;
        input rx_data_s;
        input done_m;
        input busy_m;
        input done_s;
        input busy_s;
    endclocking
endinterface

class spi_seq_item extends uvm_sequence_item;
    rand bit   [7:0] clk_div;
    rand logic [7:0] tx_data_m;
    rand logic [7:0] tx_data_s;
    bit              start;
    logic      [7:0] rx_data_m;
    logic      [7:0] rx_data_s;
    logic            done_m;
    logic            busy_m;
    logic            done_s;
    logic            busy_s;

    `uvm_object_utils_begin(spi_seq_item)
        `uvm_field_int(clk_div, UVM_ALL_ON)
        `uvm_field_int(tx_data_m, UVM_ALL_ON)
        `uvm_field_int(tx_data_s, UVM_ALL_ON)
        `uvm_field_int(start, UVM_ALL_ON)
        `uvm_field_int(rx_data_m, UVM_ALL_ON)
        `uvm_field_int(rx_data_s, UVM_ALL_ON)
        `uvm_field_int(done_m, UVM_ALL_ON)
        `uvm_field_int(busy_m, UVM_ALL_ON)
        `uvm_field_int(done_s, UVM_ALL_ON)
        `uvm_field_int(busy_s, UVM_ALL_ON)
    `uvm_object_utils_end

    constraint c_div_cnt {
        clk_div == 8'd4;
    }

    function new(string name = "spi_seq_item");
        super.new(name);
    endfunction

    function string convert2string();
        return $sformatf("[Master] tx_data=0x%0h, rx_data=0x%0h [Slave] tx_data=0x%0h, rx_data=0x%0h", 
                         tx_data_m, rx_data_m, tx_data_s, rx_data_s);
    endfunction
endclass

class spi_seq extends uvm_sequence #(spi_seq_item);
    `uvm_object_utils(spi_seq)
    int num_trans = 10;

    function new(string name = "spi_seq");
        super.new(name);
    endfunction

    task body();
        spi_seq_item item;
        repeat(num_trans) begin
            item = spi_seq_item::type_id::create("item");
            start_item(item);
            if (!item.randomize()) begin
                `uvm_fatal(get_type_name(), "randomization failed")
            end
            `uvm_info(get_type_name(), $sformatf("seq send: %s", item.convert2string()), UVM_MEDIUM)
            finish_item(item);
        end
    endtask
endclass


class spi_driver extends uvm_driver #(spi_seq_item);
    `uvm_component_utils(spi_driver)
    virtual spi_if s_if;

    function new(string name = "spi_drv", uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual spi_if)::get(this, "", "s_if", s_if)) begin
            `uvm_fatal(get_type_name(), "cannot access to interface")
        end
    endfunction

    virtual task run_phase(uvm_phase phase);
        spi_seq_item item;

        wait (s_if.rst === 1'b0);
        @(s_if.drv_cb);

        forever begin
            seq_item_port.get_next_item(item);

            // 1. 이전 통신 잔여 신호(busy, done)가 모두 0이 될 때까지 대기
            while(s_if.drv_cb.busy_m !== 1'b0 || s_if.drv_cb.busy_s !== 1'b0 || 
                  s_if.drv_cb.done_m !== 1'b0 || s_if.drv_cb.done_s !== 1'b0)
                @(s_if.drv_cb);

            s_if.drv_cb.clk_div   <= item.clk_div;
            s_if.drv_cb.tx_data_m <= item.tx_data_m;
            s_if.drv_cb.tx_data_s <= item.tx_data_s;
            s_if.drv_cb.start     <= 1'b1;

            // 2. 통신 시작 확인 및 마스터 시작 로그 출력
            while (s_if.drv_cb.busy_m !== 1'b1) @(s_if.drv_cb);
            s_if.drv_cb.start <= 1'b0;
            `uvm_info(get_type_name(), $sformatf("tx start : %s", item.convert2string()), UVM_LOW)

            // 슬레이브 동작 시작 확인 (rx_data가 비어있는 불필요한 로그는 삭제됨)
            while (s_if.drv_cb.busy_s !== 1'b1) @(s_if.drv_cb);

            // 3. 완료 신호 병렬 대기
            fork
                begin while (s_if.drv_cb.done_m !== 1'b1) @(s_if.drv_cb); end
                begin while (s_if.drv_cb.done_s !== 1'b1) @(s_if.drv_cb); end
            join

            // 4. 수신된 데이터 캡처
            item.rx_data_m = s_if.drv_cb.rx_data_m;
            item.rx_data_s = s_if.drv_cb.rx_data_s;

            // 최종 완료 로그 (이때 비로소 온전한 rx_data가 출력됨)
            `uvm_info(get_type_name(), $sformatf("tx, rx finished : %s", item.convert2string()), UVM_LOW)
            seq_item_port.item_done();
        end
    endtask
endclass


class spi_monitor extends uvm_monitor;
    `uvm_component_utils(spi_monitor)
    virtual spi_if s_if;
    uvm_analysis_port #(spi_seq_item) ap;

    function new(string name = "spi_mon", uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual spi_if)::get(this, "", "s_if", s_if)) begin
            `uvm_fatal(get_type_name(), "cannot access to interface")
        end
        ap = new("ap", this);
    endfunction

    virtual task run_phase(uvm_phase phase);
        wait (s_if.rst === 1'b0);
        @(s_if.mon_cb);

        forever begin
            spi_seq_item item = spi_seq_item::type_id::create("item", this);

            fork
                begin while (s_if.mon_cb.done_m !== 1'b1) @(s_if.mon_cb); end
                begin while (s_if.mon_cb.done_s !== 1'b1) @(s_if.mon_cb); end
            join

            item.tx_data_m = s_if.mon_cb.tx_data_m;
            item.tx_data_s = s_if.mon_cb.tx_data_s;
            item.rx_data_m = s_if.mon_cb.rx_data_m;
            item.rx_data_s = s_if.mon_cb.rx_data_s;

            `uvm_info(get_type_name(), $sformatf("Monitor Captured : %s", item.convert2string()), UVM_LOW)
            ap.write(item);

            while (s_if.mon_cb.done_m !== 1'b0 || s_if.mon_cb.done_s !== 1'b0) @(s_if.mon_cb);
        end
    endtask
endclass


class spi_agent extends uvm_agent;
    `uvm_component_utils(spi_agent)

    spi_driver drv;
    spi_monitor mon;
    uvm_sequencer #(spi_seq_item) sqr;

    function new(string name = "spi_agent", uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        drv = spi_driver::type_id::create("drv", this);
        mon = spi_monitor::type_id::create("mon", this);
        sqr = uvm_sequencer#(spi_seq_item)::type_id::create("sqr", this);
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        drv.seq_item_port.connect(sqr.seq_item_export);
    endfunction
endclass


class spi_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(spi_scoreboard)
    uvm_analysis_imp #(spi_seq_item, spi_scoreboard) ap_imp;

    int pass_cnt = 0;
    int fail_cnt = 0;

    function new(string name = "spi_scb", uvm_component parent);
        super.new(name, parent);
        ap_imp = new("ap_imp", this);
    endfunction

    function void write(spi_seq_item item);
        if ((item.tx_data_m == item.rx_data_s) && (item.rx_data_m == item.tx_data_s)) begin
            `uvm_info(get_type_name(), $sformatf("[PASS] M->S : 0x%0h, S->M : 0x%0h", item.tx_data_m, item.tx_data_s), UVM_MEDIUM)
            pass_cnt++;
        end else begin
            `uvm_error(get_type_name(), $sformatf("[FAIL] M->S : 0x%0h -> 0x%0h, S->M : 0x%0h -> 0x%0h", 
                       item.tx_data_m, item.rx_data_s, item.tx_data_s, item.rx_data_m))
            fail_cnt++;
        end
    endfunction

    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info(get_type_name(), $sformatf("\n\n ====== Scoreboard Result ====== "), UVM_LOW)
        `uvm_info(get_type_name(), $sformatf("pass_cnt = %0d/%0d", pass_cnt, pass_cnt + fail_cnt), UVM_LOW)
        `uvm_info(get_type_name(), $sformatf("fail_cnt = %0d/%0d", fail_cnt, pass_cnt + fail_cnt), UVM_LOW)
        `uvm_info(get_type_name(), $sformatf(" ====== Scoreboard Result ====== \n\n"), UVM_LOW)
    endfunction
endclass

class spi_coverage extends uvm_subscriber #(spi_seq_item);
    `uvm_component_utils(spi_coverage)

    logic [7:0] cov_tx_data_m, cov_tx_data_s;

    covergroup cg_data;
        cp_tx_data_m: coverpoint cov_tx_data_m {
            bins zero     = {8'h00};
            bins alt_01   = {8'h55};
            bins alt_10   = {8'haa};
            bins lsb_only = {8'h01};
            bins msb_only = {8'h80};
            bins range0   = {[8'h00:8'h0f]};
            bins range1   = {[8'h10:8'h1f]};
            bins range2   = {[8'h20:8'h2f]};
            bins range3   = {[8'h30:8'h3f]};
            bins range4   = {[8'h40:8'h4f]};
            bins range5   = {[8'h50:8'h5f]};
            bins range6   = {[8'h60:8'h6f]};
            bins range7   = {[8'h70:8'h7f]};
            bins range8   = {[8'h80:8'h8f]};
            bins range9   = {[8'h90:8'h9f]};
            bins rangea   = {[8'ha0:8'haf]};
            bins rangeb   = {[8'hb0:8'hbf]};
            bins rangec   = {[8'hc0:8'hcf]};
            bins ranged   = {[8'hd0:8'hdf]};
            bins rangee   = {[8'he0:8'hef]};
            bins rangef   = {[8'hf0:8'hff]};
        }
        cp_tx_data_s: coverpoint cov_tx_data_s {
            bins zero     = {8'h00};
            bins alt_01   = {8'h55};
            bins alt_10   = {8'haa};
            bins lsb_only = {8'h01};
            bins msb_only = {8'h80};
            bins range0   = {[8'h00:8'h0f]};
            bins range1   = {[8'h10:8'h1f]};
            bins range2   = {[8'h20:8'h2f]};
            bins range3   = {[8'h30:8'h3f]};
            bins range4   = {[8'h40:8'h4f]};
            bins range5   = {[8'h50:8'h5f]};
            bins range6   = {[8'h60:8'h6f]};
            bins range7   = {[8'h70:8'h7f]};
            bins range8   = {[8'h80:8'h8f]};
            bins range9   = {[8'h90:8'h9f]};
            bins rangea   = {[8'ha0:8'haf]};
            bins rangeb   = {[8'hb0:8'hbf]};
            bins rangec   = {[8'hc0:8'hcf]};
            bins ranged   = {[8'hd0:8'hdf]};
            bins rangee   = {[8'he0:8'hef]};
            bins rangef   = {[8'hf0:8'hff]};
        }
    endgroup

    function new(string name = "spi_coverage", uvm_component parent);
        super.new(name, parent);
        cg_data = new();
    endfunction

    function void write(spi_seq_item item);
        cov_tx_data_m = item.tx_data_m;
        cov_tx_data_s = item.tx_data_s;
        cg_data.sample();
    endfunction

    function void report_phase(uvm_phase phase);
        `uvm_info(get_type_name(), "\n\n ===== Coverage Report ===== ", UVM_LOW)
        `uvm_info(get_type_name(), $sformatf("coverage cg_data=%.1f%%", cg_data.get_coverage()), UVM_LOW)
        `uvm_info(get_type_name(), $sformatf("coverage tx_data_m=%.1f%%", cg_data.cp_tx_data_m.get_coverage()), UVM_LOW)
        `uvm_info(get_type_name(), $sformatf("coverage tx_data_s=%.1f%%", cg_data.cp_tx_data_s.get_coverage()), UVM_LOW)
        `uvm_info(get_type_name(), " ===== Coverage Report ===== \n\n", UVM_LOW)
    endfunction
endclass


class spi_env extends uvm_env;
    `uvm_component_utils(spi_env)

    spi_agent agt;
    spi_scoreboard scb;
    spi_coverage cov;

    function new(string name = "spi_env", uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        agt = spi_agent::type_id::create("agt", this);
        scb = spi_scoreboard::type_id::create("scb", this);
        cov = spi_coverage::type_id::create("cov", this);
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        agt.mon.ap.connect(scb.ap_imp);
        agt.mon.ap.connect(cov.analysis_export);
    endfunction
endclass

class spi_test extends uvm_test;
    `uvm_component_utils(spi_test)

    spi_env env;

    function new(string name = "spi_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        env = spi_env::type_id::create("env", this);
    endfunction

    virtual task run_phase(uvm_phase phase);
        spi_seq seq;
        phase.raise_objection(this);
        seq = spi_seq::type_id::create("seq", this);
        seq.num_trans = 1000;
        seq.start(env.agt.sqr);
        phase.drop_objection(this);
    endtask
endclass


module tb_spi_uvm ();

    bit clk;
    bit rst;
    bit cpol, cpha;

    wire sclk;
    wire mosi;
    wire miso;
    wire cs_n;

    spi_if s_if (
        clk,
        rst
    );

    initial begin
        cpol = 1'b0;
        cpha = 1'b0;
    end

    spi_master SPI_M (
        .clk     (clk),
        .reset   (rst),
        .cpol    (cpol),
        .cpha    (cpha),
        .clk_div (s_if.clk_div),
        .start   (s_if.start),
        .miso    (miso),
        .mosi    (mosi),
        .sclk    (sclk),
        .cs_n    (cs_n),
        .tx_data (s_if.tx_data_m),
        .rx_data (s_if.rx_data_m),
        .done    (s_if.done_m),
        .busy    (s_if.busy_m)
    );

    spi_slave SPI_S (
        .clk     (clk),
        .rst     (rst),
        .miso    (miso),
        .mosi    (mosi),
        .sclk    (sclk),
        .cs_n    (cs_n),
        .tx_data (s_if.tx_data_s),
        .rx_data (s_if.rx_data_s),
        .done    (s_if.done_s),
        .busy    (s_if.busy_s)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0;
        rst = 1;
        repeat(3) @(posedge clk);
        rst = 0;
        @(posedge clk);
    end

    initial begin
        uvm_config_db#(virtual spi_if)::set(null, "*", "s_if", s_if);
        run_test("spi_test");

        #100;
        $finish;
    end

    initial begin
        $fsdbDumpfile("novas.fsdb");
        $fsdbDumpvars(0, tb_spi_uvm, "+all");
    end
endmodule