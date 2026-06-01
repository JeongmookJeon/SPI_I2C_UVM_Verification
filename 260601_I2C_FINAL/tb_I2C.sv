`include "uvm_macros.svh"
import uvm_pkg::*;

`timescale 1ns / 1ps




interface i2c_if (
    input bit clk,
    input bit rst
);
    logic [7:0] tx_data_m;
    logic [7:0] tx_data_s;
    logic [7:0] rx_data_m;
    logic [7:0] rx_data_s;

    
    logic       cmd_start;
    logic       cmd_write;
    logic       cmd_read;
    logic       cmd_stop;
    logic       ack_in;

    logic       done_m;
    logic       busy_m;
    logic       done_s;
    logic       busy_s;

    clocking drv_cb @(posedge clk);
        default input #1step output #0;
        output tx_data_m;
        output tx_data_s;
        output cmd_start;
        output cmd_write;
        output cmd_read;
        output cmd_stop;
        output ack_in;
        input  rx_data_m;
        input  rx_data_s;
        input  done_m;
        input  busy_m;
        input  done_s;
        input  busy_s;
    endclocking

    clocking mon_cb @(posedge clk);
        default input #1step;
        input tx_data_m;
        input tx_data_s;
        input cmd_start;
        input cmd_write;
        input cmd_read;
        input cmd_stop;
        input ack_in;
        input rx_data_m;
        input rx_data_s;
        input done_m;
        input busy_m;
        input done_s;
        input busy_s;
    endclocking
endinterface




class i2c_seq_item extends uvm_sequence_item;
    rand logic [7:0] tx_data_m;
    rand logic [7:0] tx_data_s;
    logic      [7:0] rx_data_m;
    logic      [7:0] rx_data_s;
    logic            done_m;
    logic            busy_m;
    logic            done_s;
    logic            busy_s;

    `uvm_object_utils_begin(i2c_seq_item)
        `uvm_field_int(tx_data_m, UVM_ALL_ON)
        `uvm_field_int(tx_data_s, UVM_ALL_ON)
        `uvm_field_int(rx_data_m, UVM_ALL_ON)
        `uvm_field_int(rx_data_s, UVM_ALL_ON)
        `uvm_field_int(done_m, UVM_ALL_ON)
        `uvm_field_int(busy_m, UVM_ALL_ON)
        `uvm_field_int(done_s, UVM_ALL_ON)
        `uvm_field_int(busy_s, UVM_ALL_ON)
    `uvm_object_utils_end

    function new(string name = "i2c_seq_item");
        super.new(name);
    endfunction

    function string convert2string();
        return $sformatf("[Master] tx_data=0x%0h, rx_data=0x%0h [Slave] tx_data=0x%0h, rx_data=0x%0h", 
                         tx_data_m, rx_data_m, tx_data_s, rx_data_s);
    endfunction
endclass




class i2c_seq extends uvm_sequence #(i2c_seq_item);
    `uvm_object_utils(i2c_seq)
    int num_trans = 1000;

    function new(string name = "i2c_seq");
        super.new(name);
    endfunction

    task body();
        i2c_seq_item item;
        repeat(num_trans) begin
            item = i2c_seq_item::type_id::create("item");
            start_item(item);
            if (!item.randomize()) begin
                `uvm_fatal(get_type_name(), "randomization failed")
            end
            `uvm_info(get_type_name(), $sformatf("seq send: %s", item.convert2string()), UVM_MEDIUM)
            finish_item(item);
        end
    endtask
endclass




class i2c_driver extends uvm_driver #(i2c_seq_item);
    `uvm_component_utils(i2c_driver)
    virtual i2c_if s_if;

    function new(string name = "i2c_drv", uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual i2c_if)::get(this, "", "s_if", s_if)) begin
            `uvm_fatal(get_type_name(), "cannot access to interface")
        end
    endfunction

    virtual task run_phase(uvm_phase phase);
        i2c_seq_item item;
        localparam SLA_ADDR = 7'h12; 

        
        s_if.drv_cb.cmd_start <= 1'b0;
        s_if.drv_cb.cmd_write <= 1'b0;
        s_if.drv_cb.cmd_read  <= 1'b0;
        s_if.drv_cb.cmd_stop  <= 1'b0;
        s_if.drv_cb.tx_data_m <= 8'h00;
        s_if.drv_cb.tx_data_s <= 8'h00;
        s_if.drv_cb.ack_in    <= 1'b0;

        wait (s_if.rst === 1'b0);
        @(s_if.drv_cb);

        forever begin
            seq_item_port.get_next_item(item);

            while(s_if.drv_cb.busy_m !== 1'b0) @(s_if.drv_cb);

            s_if.drv_cb.tx_data_s <= item.tx_data_s; 

            
            
            
            s_if.drv_cb.cmd_start <= 1'b1;
            @(s_if.drv_cb);
            s_if.drv_cb.cmd_start <= 1'b0;
            while (s_if.drv_cb.done_m !== 1'b1) @(s_if.drv_cb);

            
            s_if.drv_cb.cmd_write <= 1'b1;
            s_if.drv_cb.tx_data_m <= {SLA_ADDR, 1'b0};
            @(s_if.drv_cb);
            s_if.drv_cb.cmd_write <= 1'b0;
            while (s_if.drv_cb.done_m !== 1'b1) @(s_if.drv_cb);

            
            s_if.drv_cb.cmd_write <= 1'b1;
            s_if.drv_cb.tx_data_m <= item.tx_data_m;
            @(s_if.drv_cb);
            s_if.drv_cb.cmd_write <= 1'b0;
            while (s_if.drv_cb.done_m !== 1'b1) @(s_if.drv_cb);

            
            s_if.drv_cb.cmd_stop <= 1'b1;
            @(s_if.drv_cb);
            s_if.drv_cb.cmd_stop <= 1'b0;
            while (s_if.drv_cb.done_m !== 1'b1) @(s_if.drv_cb);

            repeat(5) @(s_if.drv_cb);

            
            
            
            s_if.drv_cb.cmd_start <= 1'b1;
            @(s_if.drv_cb);
            s_if.drv_cb.cmd_start <= 1'b0;
            while (s_if.drv_cb.done_m !== 1'b1) @(s_if.drv_cb);

            
            s_if.drv_cb.cmd_write <= 1'b1;
            s_if.drv_cb.tx_data_m <= {SLA_ADDR, 1'b1};
            @(s_if.drv_cb);
            s_if.drv_cb.cmd_write <= 1'b0;
            while (s_if.drv_cb.done_m !== 1'b1) @(s_if.drv_cb);

            
            s_if.drv_cb.cmd_read <= 1'b1;
            s_if.drv_cb.ack_in   <= 1'b1; 
            @(s_if.drv_cb);
            s_if.drv_cb.cmd_read <= 1'b0;
            while (s_if.drv_cb.done_m !== 1'b1) @(s_if.drv_cb);

            
            s_if.drv_cb.cmd_stop <= 1'b1;
            @(s_if.drv_cb);
            s_if.drv_cb.cmd_stop <= 1'b0;
            while (s_if.drv_cb.done_m !== 1'b1) @(s_if.drv_cb);

            repeat(5) @(s_if.drv_cb);

            
            item.rx_data_m = s_if.drv_cb.rx_data_m;
            item.rx_data_s = s_if.drv_cb.rx_data_s;

            `uvm_info(get_type_name(), $sformatf("tx, rx finished : %s", item.convert2string()), UVM_LOW)
            seq_item_port.item_done();
        end
    endtask
endclass




class i2c_monitor extends uvm_monitor;
    `uvm_component_utils(i2c_monitor)
    virtual i2c_if s_if;
    uvm_analysis_port #(i2c_seq_item) ap;

    function new(string name = "i2c_mon", uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual i2c_if)::get(this, "", "s_if", s_if)) begin
            `uvm_fatal(get_type_name(), "cannot access to interface")
        end
        ap = new("ap", this);
    endfunction

    virtual task run_phase(uvm_phase phase);
        wait (s_if.rst === 1'b0);
        @(s_if.mon_cb);
        forever begin
            i2c_seq_item item = i2c_seq_item::type_id::create("item", this);

            
            while (s_if.mon_cb.cmd_start !== 1'b1) @(s_if.mon_cb);
            while (s_if.mon_cb.cmd_write !== 1'b1) @(s_if.mon_cb); 
            while (s_if.mon_cb.done_m !== 1'b1) @(s_if.mon_cb); 
            
            while (s_if.mon_cb.cmd_write !== 1'b1) @(s_if.mon_cb); 
            item.tx_data_m = s_if.mon_cb.tx_data_m;
            item.tx_data_s = s_if.mon_cb.tx_data_s;
            while (s_if.mon_cb.done_m !== 1'b1) @(s_if.mon_cb);
            
            while (s_if.mon_cb.cmd_stop !== 1'b1) @(s_if.mon_cb);
            while (s_if.mon_cb.done_m !== 1'b1) @(s_if.mon_cb);
            
            
            while (s_if.mon_cb.cmd_start !== 1'b1) @(s_if.mon_cb);
            while (s_if.mon_cb.done_m !== 1'b1) @(s_if.mon_cb);

            while (s_if.mon_cb.cmd_write !== 1'b1) @(s_if.mon_cb);
            while (s_if.mon_cb.done_m !== 1'b1) @(s_if.mon_cb);

            while (s_if.mon_cb.cmd_read !== 1'b1) @(s_if.mon_cb);
            while (s_if.mon_cb.done_m !== 1'b1) @(s_if.mon_cb);

            while (s_if.mon_cb.cmd_stop !== 1'b1) @(s_if.mon_cb);
            while (s_if.mon_cb.done_m !== 1'b1) @(s_if.mon_cb);

            
            item.rx_data_m = s_if.mon_cb.rx_data_m;
            item.rx_data_s = s_if.mon_cb.rx_data_s;

            `uvm_info(get_type_name(), $sformatf("Monitor Captured : %s", item.convert2string()), UVM_LOW)
            ap.write(item);
        end
    endtask
endclass




class i2c_agent extends uvm_agent;
    `uvm_component_utils(i2c_agent)

    i2c_driver drv;
    i2c_monitor mon;
    uvm_sequencer #(i2c_seq_item) sqr;

    function new(string name = "i2c_agent", uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        drv = i2c_driver::type_id::create("drv", this);
        mon = i2c_monitor::type_id::create("mon", this);
        sqr = uvm_sequencer#(i2c_seq_item)::type_id::create("sqr", this);
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        drv.seq_item_port.connect(sqr.seq_item_export);
    endfunction
endclass




class i2c_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(i2c_scoreboard)
    uvm_analysis_imp #(i2c_seq_item, i2c_scoreboard) ap_imp;

    int pass_cnt = 0;
    int fail_cnt = 0;

    function new(string name = "i2c_scb", uvm_component parent);
        super.new(name, parent);
        ap_imp = new("ap_imp", this);
    endfunction

    function void write(i2c_seq_item item);
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




class i2c_coverage extends uvm_subscriber #(i2c_seq_item);
    `uvm_component_utils(i2c_coverage)

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

    function new(string name = "i2c_coverage", uvm_component parent);
        super.new(name, parent);
        cg_data = new();
    endfunction

    function void write(i2c_seq_item item);
        cov_tx_data_m = item.tx_data_m;
        cov_tx_data_s = item.tx_data_s;
        cg_data.sample();
    endfunction

    function void report_phase(uvm_phase phase);
        `uvm_info(get_type_name(), "\n\n ===== 커버리지 리포트 ===== ", UVM_LOW)
        `uvm_info(get_type_name(), $sformatf("cg_data 커버리지=%.1f%%", cg_data.get_coverage()), UVM_LOW)
        `uvm_info(get_type_name(), $sformatf("tx_data_m 커버리지=%.1f%%", cg_data.cp_tx_data_m.get_coverage()), UVM_LOW)
        `uvm_info(get_type_name(), $sformatf("tx_data_s 커버리지=%.1f%%", cg_data.cp_tx_data_s.get_coverage()), UVM_LOW)
        `uvm_info(get_type_name(), " ===== 커버리지 리포트 ===== \n\n", UVM_LOW)
    endfunction
endclass




class i2c_env extends uvm_env;
    `uvm_component_utils(i2c_env)

    i2c_agent      agt;
    i2c_scoreboard scb;
    i2c_coverage   cov;

    function new(string name = "i2c_env", uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        agt = i2c_agent::type_id::create("agt", this);
        scb = i2c_scoreboard::type_id::create("scb", this);
        cov = i2c_coverage::type_id::create("cov", this);
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        agt.mon.ap.connect(scb.ap_imp);
        agt.mon.ap.connect(cov.analysis_export);
    endfunction
endclass

class i2c_test extends uvm_test;
    `uvm_component_utils(i2c_test)
    i2c_env env;

    function new(string name = "i2c_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        env = i2c_env::type_id::create("env", this);
    endfunction

    virtual task run_phase(uvm_phase phase);
        i2c_seq seq;
        phase.raise_objection(this);
        seq = i2c_seq::type_id::create("seq", this);
        seq.num_trans = 1000;
        seq.start(env.agt.sqr);
        phase.drop_objection(this);
    endtask
endclass




module tb_i2c_uvm ();

    bit clk;
    bit rst;

    wire scl;
    wire sda;
    
    
    pullup(scl);
    pullup(sda);

    i2c_if s_if (
        clk,
        rst
    );

    
    
    
    I2C_MASTER I2C_M (
        .clk       (clk),
        .rst_n     (~rst), 
        .cmd_start (s_if.cmd_start),
        .cmd_write (s_if.cmd_write),
        .cmd_read  (s_if.cmd_read),
        .cmd_stop  (s_if.cmd_stop),
        .tx_data   (s_if.tx_data_m),
        .ack_in    (s_if.ack_in),
        .rx_data   (s_if.rx_data_m),
        .done      (s_if.done_m),
        .ack_out   (),
        .busy      (s_if.busy_m),
        .scl       (scl),
        .sda       (sda)
    );

    I2C_SLAVE I2C_S (
        .clk       (clk),
        .reset     (rst),  
        .tx_data   (s_if.tx_data_s),
        .rx_data   (s_if.rx_data_s),
        .done      (s_if.done_s),
        .busy      (s_if.busy_s),
        .scl       (scl),
        .sda       (sda)
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
        uvm_config_db#(virtual i2c_if)::set(null, "*", "s_if", s_if);
        run_test("i2c_test");

        #100;
        $finish;
    end

    initial begin
        $fsdbDumpfile("novas.fsdb");
        $fsdbDumpvars(0, tb_i2c_uvm, "+all");
    end
endmodule