module tb_cache;

// each line goes as follows:
// [31:0]rs1, [31:0]imm, [31:0]store_data, [2:0]funct3, ld_st, [5:0] id, [5:0] addr_ptr,
// 112 total bits per instruction


    logic [12:0]read_addr;//, write_addr;
    logic reset, clk;

    int cache_len = 1152;

    logic [120:0] inst_mem [300:0];

  
    logic [31:0] cur_rs1, cur_imm, cur_store_data;
    logic [2:0] cur_funct3;
    logic [5:0] cur_id, cur_addr;
    logic new_inst, cur_load_store;

    logic [5:0] broad_addr, broad_id, cache_addr, cache_id;
    logic [31:0] broad_val, cache_val;
    logic broad_true, cache_true, stall;
    parameter NUM_INSTRUCTIONS = 255;
    logic [8:0] count;

    integer file_handle;
    mem_unit mem_unit_inst (
        // Inputs
        .rs1              (cur_rs1),
        .imm              (cur_imm),
        .store_data       (cur_store_data),
        .funct3_in        (cur_funct3),
        .load_store       (cur_load_store),
        .id_in            (cur_id),
        .addr_in          (cur_addr),
        .new_inst         (new_inst),
        .clk              (clk),
        .reset            (reset),
        
        // Outputs
        .broadcast_addr   (broad_addr),
        .broadcast_id     (broad_id),
        .broadcast_val    (broad_val),
        .broadcast_true   (broad_true),
        .cache_broad_addr (cache_addr),
        .cache_broad_id   (cache_id),
        .cache_broad_val  (cache_val),
        .cache_broad_true (cache_true),
        .stall            (stall)
    );




    initial begin
        clk = 0;
        forever #10 clk = ~clk;
    end
    
    initial begin
        read_addr = 0; reset = 0; 
        #20 reset = 1;


        // #5000 $display("TIMEOUT REACHED. FORCING EXIT."); $finish;

    end



    initial begin
        for (int i = 0; i < 300; i++) begin
            inst_mem[i] = '0;
        end
        $readmemh("./micro_3/mem_test.hex", inst_mem, 0, NUM_INSTRUCTIONS);
        file_handle = $fopen("./micro_3/sim_results.csv", "w");

        if (file_handle == 0) begin
            $error("Failed to open the log file!");
        end     


    end

    task finish_sim;
        $fclose(file_handle);
        $display("Simulation finished. Results saved to sim_results.csv");
        $finish;
    endtask

    always_ff @(posedge clk or negedge reset) begin
        if (~reset) begin
            cur_rs1 <= 0;
            cur_imm <= 0;
            cur_store_data <= 0;
            cur_funct3 <= 0;
            cur_load_store <= 0;
            cur_id <= 0;
            cur_addr <= 0;
            new_inst <= 0;
            count <= 0;
        end else begin
            if (count > (NUM_INSTRUCTIONS + 30)) begin
                new_inst <= 0;
                finish_sim();
            end else begin

                if(~stall)begin
                    if(count < NUM_INSTRUCTIONS + 1)begin
                        new_inst       <= inst_mem[count][112];
                        cur_rs1        <= inst_mem[count][111:80];
                        cur_imm        <= inst_mem[count][79:48];
                        cur_store_data <= inst_mem[count][47:16];
                        cur_funct3     <= inst_mem[count][15:13];
                        cur_load_store <= inst_mem[count][12];
                        cur_id         <= inst_mem[count][11:6];
                        cur_addr       <= inst_mem[count][5:0];
                    end else begin
                        new_inst       <= 0;
                        cur_rs1        <= 0;
                        cur_imm        <= 0;
                        cur_store_data <= 0;
                        cur_funct3     <= 0;
                        cur_load_store <= 0;
                        cur_id         <= 0;
                        cur_addr       <= 0;
                    end
                    count++;
                end else begin
                    new_inst <= 0;
                end

                // // Check broadcast outputs
                if ($isunknown(broad_true)) begin
                    $display("Broad true is X/Z at %0t", $time);
                end else if (broad_true && !$isunknown({broad_id, broad_addr, broad_val})) begin
                    $fdisplay(file_handle, "%h,%h,%h", broad_id, broad_addr, broad_val);
                end  else if (broad_true) begin
                    $display("Broad broadcast contains X/Z at %0t", $time);
                end

                if ($isunknown(cache_true)) begin
                    $display("Cache true is X/Z at %0t", $time);
                end else if (cache_true && !$isunknown({cache_id, cache_addr, cache_val})) begin
                    $fdisplay(file_handle, "%h,%h,%h", cache_id, cache_addr, cache_val);
                end else if (cache_true) begin
                    $display("Cache broadcast contains X/Z at %0t", $time);
                end

            end
        end
    end



    initial begin
        $monitor ($time, " | count: %0d | rs1: %h | imm: %h | ld_st: %b | broad_true: %b", 
                  count, cur_rs1, cur_imm, cur_load_store, broad_true);
    end


endmodule

