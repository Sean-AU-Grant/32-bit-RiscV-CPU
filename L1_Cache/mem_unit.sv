module mem_unit(input logic [31:0] rs1,
                input logic [31:0] imm,
                input logic [31:0] store_data,
                input logic [2:0] funct3_in,
                input logic load_store,
                input logic [5:0] id_in,
                input logic [5:0] addr_in,
                input logic new_inst,
                input logic clk,
                input logic reset,
                output logic [5:0] broadcast_addr,
                output logic [5:0] broadcast_id,
                output logic [31:0] broadcast_val,
                output logic broadcast_true,
                output logic [5:0] cache_broad_addr,
                output logic [5:0] cache_broad_id,
                output logic [31:0] cache_broad_val,
                output logic cache_broad_true,
                output logic stall);


 


    logic [31:0] mem_addr;
    logic [31:0] mem_addr_cache;
    logic [31:0] store_data_cache;
    logic [2:0] funct3_cache;
    logic [5:0] inst_id_cache;
    logic [5:0] inst_addr_cache;
    logic cache_ls;
    logic new_cache_inst;
    logic [12:0] ctc_addr;
    logic [2:0] ctc_funct3;
    logic write_cache;
    logic [31:0] ctc_val;

    logic [31:0] fifo_addr;
    logic [31:0] fifo_data;
    logic [2:0] fifo_funct3;
    logic fifo_new_inst;

    logic empty;
    logic [31:0] fifo_addr_out;
    logic [31:0] fifo_data_out;
    logic [2:0] fifo_funct3_out;


    logic [25:0] block_addr;
    logic [12:0] cache_write_addr;

    logic read_fifo, fifo_full, trig_mem_read, block_read_done;
    
    logic [31:0] mtc_data;
    logic [12:0] mtc_addr;
    logic mtc_write;
    logic read_busy, st_ld_full;

    logic stall_forward, stall_forward_comb;

    assign stall = st_ld_full;


    addr_adder ADD(.rs1(rs1), .imm(imm), .addr(mem_addr));


    st_ld_forward FORWARD(
        .mem_addr           (mem_addr), 
        .funct3             (funct3_in), 
        .load_store         (load_store), 
        .store_data         (store_data), 
        .new_inst           (new_inst), 
        .inst_id_in         (id_in), 
        .addr_in            (addr_in),
        .stall_forward      (stall_forward),
        .stall_forward_comb (stall_forward_comb),
        .clk                (clk), 
        .reset              (reset), 
        .broadcast_addr     (broadcast_addr), 
        .broadcast_val      (broadcast_val), 
        .broadcast_true     (broadcast_true), 
        .broadcast_id       (broadcast_id),
        .mem_addr_cache     (mem_addr_cache),
        .store_data_cache   (store_data_cache),
        .funct3_cache       (funct3_cache),
        .inst_id_cache      (inst_id_cache),
        .inst_addr_cache    (inst_addr_cache),
        .cache_ls           (cache_ls),
        .new_cache_inst     (new_cache_inst),
        .fifo_addr_out          (fifo_addr),
        .fifo_data_out          (fifo_data),
        .fifo_funct3_out        (fifo_funct3),
        .fifo_new_inst      (fifo_new_inst),
        .full               (st_ld_full)
    );



    cache_controller CONT (
        .addr               (mem_addr_cache),
        .addr_in            (inst_addr_cache),
        .id_in              (inst_id_cache),
        .load_store         (cache_ls),
        .write_val          (store_data_cache),
        .funct3_in          (funct3_cache),
        .new_inst           (new_cache_inst),
        .mem_trans_done     (block_read_done),
        .clk                (clk),
        .reset              (reset),
        .cache_addr_out     (ctc_addr),
        .funct3             (ctc_funct3),
        .write_cache        (write_cache),
        .write_cache_val    (ctc_val),
        .broadcast_true     (cache_broad_true),
        .broadcast_id       (cache_broad_id),
        .broadcast_addr     (cache_broad_addr),
        .read_busy          (read_busy),
        .mem_cont_read_addr (block_addr),
        .mem_cont_write_addr(cache_write_addr),
        .trig_mem_read      (trig_mem_read),
        .stall_forward      (stall_forward),
        .stall_forward_comb (stall_forward_comb)
    );




    l1_cache L1(
        .cont_addr(ctc_addr), 
        .funct3(ctc_funct3), 
        .write_data(ctc_val), 
        .mem_data(mtc_data), 
        .mem_data_addr(mtc_addr), 
        .write_mem_data(mtc_write),
        .write(write_cache), 
        .clk(clk), 
        .reset(reset), 
        .read_data(cache_broad_val)
    );


    store_fifo FIFO (
        .new_addr   (fifo_addr),
        .new_data   (fifo_data),
        .new_funct3 (fifo_funct3),
        .new_inst   (fifo_new_inst),
        .send_line  (read_fifo),
        .read_block (trig_mem_read),
        .clk        (clk),
        .reset      (reset),
        .full       (fifo_full),
        .empty      (empty),
        .addr_out   (fifo_addr_out),
        .data_out   (fifo_data_out),
        .funct3_out (fifo_funct3_out)
    );




    mem_cont MEM_CONT (
        .fifo_data      (fifo_data_out),
        .fifo_addr      (fifo_addr_out),
        .fifo_funct3    (fifo_funct3),
        .fifo_valid     (~empty),
        .block_addr     (block_addr),
        .cache_addr     (cache_write_addr),
        .read_block     (trig_mem_read),
        .reset          (reset),
        .clk            (clk),
        .read_data      (mtc_data),
        .read_addr      (mtc_addr),
        .write_cache    (mtc_write),
        .read_fifo      (read_fifo),
        .read_done      (block_read_done)
    );


endmodule



package defs_pkg;
    typedef enum logic [2:0] {BYTE, HALF, WORD, BYTE_U, HALF_U} funct3_types;
endpackage

import defs_pkg::*;
module l1_cache(input logic [12:0] cont_addr,
                input logic [2:0] funct3,
                input logic [31:0] write_data,
                input logic [31:0] mem_data,
                input logic [12:0] mem_data_addr,
                input logic write_mem_data,
                input logic write,
                input logic clk,
                input logic reset,
                output logic [31:0] read_data 
                );

    // 16 words a block * 72 total blocks = 1152 32 bit words
    localparam int cache_len = 1152;
    logic [31:0] cache [cache_len -1:0];
    logic [4:0] byte_addr;
    assign byte_addr = cont_addr[1:0] << 3;



    always_ff @(posedge clk or negedge reset)begin
        if(~reset)begin
            for(int i = 0; i < cache_len; i++)begin
                cache[i] <= 0;
            end
        end else begin
            if(write)begin
                case(funct3_types'(funct3))
                    BYTE: cache[cont_addr[12:2]][byte_addr +: 8] <= write_data[7:0];
                    HALF: cache[cont_addr[12:2]][byte_addr +: 16] <= write_data[15:0];
                    WORD: cache[cont_addr[12:2]] <= write_data;
                    default: begin end
                endcase
            end
            
            if(write_mem_data)begin
                cache[mem_data_addr[12:2]] <= mem_data;
            end

        end
    end

    always_comb begin
        case(funct3_types'(funct3))
            BYTE: read_data = {{24{cache[cont_addr[12:2]][byte_addr + 7]}}, cache[cont_addr[12:2]][byte_addr +: 8]};
            HALF: read_data = {{16{cache[cont_addr[12:2]][byte_addr + 16]}}, cache[cont_addr[12:2]][byte_addr +: 16]};
            WORD: read_data = cache[cont_addr[12:2]];
            BYTE_U: read_data = {{24{1'b0}}, cache[cont_addr[12:2]][byte_addr +: 8]};
            HALF_U: read_data = {{16{1'b0}}, cache[cont_addr[12:2]][byte_addr +: 16]};
            default: read_data = 0;

        endcase


    end

endmodule



module addr_adder(input logic [31:0] rs1,
                  input logic [31:0] imm,
                  output logic [31:0] addr);
    assign addr = rs1 + imm;

endmodule



module st_ld_forward(input logic [31:0] mem_addr,
                     input logic [2:0] funct3,
                     input logic load_store, // 1 if inst is load, 0 if inst is store
                     input logic [31:0] store_data,
                     input logic new_inst,
                     input logic [5:0] inst_id_in,
                     input logic [5:0] addr_in,
                     input logic stall_forward,
                     input logic stall_forward_comb,
                     input logic clk,
                     input logic reset,
                     output logic [5:0] broadcast_addr,
                     output logic [31:0] broadcast_val,
                     output logic broadcast_true,
                     output logic [5:0] broadcast_id,
                     output logic [31:0] mem_addr_cache,
                     output logic [31:0] store_data_cache,
                     output logic [2:0] funct3_cache,
                     output logic [5:0] inst_id_cache,
                     output logic [5:0] inst_addr_cache,
                     output logic cache_ls,
                     output logic new_cache_inst,
                     output logic [31:0] fifo_addr_out,
                     output logic [31:0] fifo_data_out,
                     output logic [2:0] fifo_funct3_out,
                     output logic fifo_new_inst,
                     output logic full
                     );

    localparam int RAM_len = 64;
    logic [5:0] front_pointer, fifo_front, fifo_tail;
    logic [6:0] fifo_cnt;
    logic [31:0] store_val_RAM [RAM_len - 1:0];
    logic [31:0] fifo_val [RAM_len -1 : 0];
    logic [5:0] fifo_id [RAM_len -1 : 0];
    logic [5:0] fifo_addr [RAM_len -1 : 0];
    logic [2:0] fifo_funct3 [RAM_len -1 : 0];
    logic [31:0] fifo_mem_addr [RAM_len -1 : 0];
    logic [RAM_len -1 : 0] fifo_ld_st;

    logic [31:0] store_mem_addr_RAM [RAM_len - 1:0];
    logic [5:0] load_forward_addr;
    logic load_local_hit, empty;
    



    always_ff @(posedge clk or negedge reset) begin
        if(~reset)begin
            front_pointer <= 0;
            fifo_addr_out <= 0;
            fifo_data_out <= 0;
            fifo_funct3_out <= 0;
            fifo_new_inst <= 0;
            broadcast_true <= 0;
            broadcast_id <= 0;
            broadcast_val <= 0;
            fifo_front <= 0;
            fifo_tail <= 0;
            fifo_cnt <= 0;
            fifo_ld_st <= 0;
            new_cache_inst <= 0;

            for(int i = 0; i < RAM_len; i++)begin
                store_val_RAM[i] <= 0;
                store_mem_addr_RAM[i] <= 0;
                fifo_val[i] <= 0;
                fifo_id[i] <= 0;
                fifo_addr[i] <= 0;
                fifo_funct3[i] <= 0;
                fifo_mem_addr[i] <= 0;

            end

        end else begin 
 
            if(new_inst && load_local_hit)begin
                broadcast_true <= 1;
                broadcast_addr <= addr_in;
                broadcast_id <= inst_id_in;
                broadcast_val <= store_val_RAM[load_forward_addr];
            end else begin
                broadcast_true <= 0;
                broadcast_addr <= 0;
                broadcast_id <= 0;
                broadcast_val <= 0;
            end


            /*
                load_local_hit: inst is a load and the mem addr is in the local ram
                new_inst:
                    if load_local_hit
                        forward from local ram
                    else 
                        if fifo empty
                            send to cache 
                        else
                            append to fifo
                
                store:
                    if new_inst and load local hit
                        broadcast from ram
                    else if new inst and not fifo empty
                        push inst to fifo



                cache:
                    if not stalled
                        if fifo empty
                            if(new_inst & ~load_local_hit)
                                cache new_inst
                        else 
                            cache from fifo
                    else 
                        dont cache
            
            */

            if((new_inst && ~load_local_hit && ~empty) || (new_inst && ~load_local_hit && stall_forward_comb) )begin
                fifo_addr[fifo_tail] <= addr_in;
                fifo_mem_addr[fifo_tail] <= mem_addr;
                fifo_funct3[fifo_tail] <= funct3;
                fifo_ld_st[fifo_tail] <= load_store;
                fifo_val[fifo_tail] <= store_data;
                fifo_id[fifo_tail] <= inst_id_in;
                fifo_tail++;
                fifo_cnt ++;
            end

            if(~stall_forward_comb)begin
                if(empty)begin
                    if(new_inst & ~load_local_hit)begin
                        funct3_cache <= funct3; 
                        inst_id_cache <= inst_id_in;
                        inst_addr_cache <= addr_in;
                        store_data_cache <= store_data;
                        mem_addr_cache <= mem_addr;
                        cache_ls <= load_store;
                        new_cache_inst <=1;
                    end else begin
                        new_cache_inst <= 0;
                    end
                end else begin
                    funct3_cache <= fifo_funct3[fifo_front]; 
                    inst_id_cache <= fifo_id[fifo_front];
                    inst_addr_cache <= fifo_addr[fifo_front];
                    store_data_cache <= fifo_val[fifo_front];
                    mem_addr_cache <= fifo_mem_addr[fifo_front];
                    cache_ls <= fifo_ld_st[fifo_front];
                    fifo_front ++;
                    fifo_cnt --;
                    new_cache_inst <=1;

                end

            end else begin
                new_cache_inst <= 0;

            end
        

            


            fifo_addr_out <= mem_addr;
            fifo_data_out <= store_data;
            fifo_funct3_out <= funct3;
            if(new_inst && ~load_store)begin
                fifo_new_inst <=1;
                store_val_RAM[front_pointer] <= store_data;
                store_mem_addr_RAM[front_pointer] <= mem_addr;
                front_pointer ++;   
            end else begin
                fifo_new_inst <= 0;

            end

            

        end

    end



    always_comb begin
        load_local_hit = 0;
        load_forward_addr = 0;
        for(logic [5:0] i = front_pointer + 1; i != front_pointer; i++ )begin
            if((store_mem_addr_RAM[i] == mem_addr) && load_store)begin
                load_local_hit = 1;
                load_forward_addr = i;                
            end
        end


        full = 0;
        if((fifo_front == fifo_tail && fifo_cnt != 0) || fifo_cnt > 62)begin
            full = 1;
        end

        empty = 0;
        if(fifo_cnt == 0)begin
            empty = 1;
        end


    end

endmodule

module cache_controller(input logic [31:0] addr,
                        input logic [5:0] addr_in,
                        input logic [5:0] id_in,
                        input logic load_store, // 1 if inst is load, 0 if inst is store
                        input logic [31:0] write_val,
                        input logic [2:0] funct3_in,
                        input logic new_inst,
                        input logic mem_trans_done,
                        input logic clk,
                        input logic reset,
                        output logic [12:0] cache_addr_out,
                        output logic [2:0] funct3,
                        output logic write_cache,
                        output logic [31:0] write_cache_val,
                        output logic broadcast_true,
                        output logic [5:0] broadcast_id,
                        output logic [5:0] broadcast_addr,
                        output logic read_busy,
                        output logic [25:0] mem_cont_read_addr,
                        output logic [12:0] mem_cont_write_addr,
                        output logic trig_mem_read,
                        output logic stall_forward,
                        output logic stall_forward_comb
                        );


        //address: |21 tag bits|5 idx bits|6 byte addr bits|

    localparam int way_len = 17; // + 1
    localparam int tag_len = 21; // +1
    localparam int offset_len = 11; //+1 
    localparam int idx_len = 4; // +1
    localparam int byte_bits = 5; // +1
    localparam int sq_len = 31; //+1
        //store queue
            logic [31:0] sq_data [sq_len:0];
            logic [12:0] sq_addr [sq_len:0];
            logic [2:0] sq_funct3 [sq_len:0];
            logic [3:0] sq_found [sq_len:0];
            logic [idx_len:0] sq_idx [sq_len:0];
            logic [4:0] front_pointer, tail_pointer;


            //Way 1
            logic [way_len:0] way1_valid;
            logic [tag_len:0] way1_tag [way_len:0];
            logic [offset_len:0] way1_data_offset [way_len:0];

            //Way 2
            logic [way_len:0] way2_valid;
            logic [tag_len:0] way2_tag [way_len:0];
            logic [offset_len:0] way2_data_offset [way_len:0];

            //Way 3
            logic [way_len:0] way3_valid;
            logic [tag_len:0] way3_tag [way_len:0];
            logic [offset_len:0] way3_data_offset [way_len:0];

            //Way 4
            logic [way_len:0] way4_valid;
            logic [tag_len:0] way4_tag [way_len:0];
            logic [offset_len:0] way4_data_offset [way_len:0];

            logic [3:0] way_vict [way_len:0];
            logic [3:0] way_next_vict [way_len:0];


                //Cache Layout
                /*
                    Max Addr: 4608 (1152 4-byte words)
                        way 4: idx 17 = 4544                   


                              idx 2 = 128 
                              idx 1 = 64 
                       way 1: idx 0 = 0 
                    Min Addr: 0x0
                */

                //VICTim = 10, next victim = 01, ordinary = 00

    typedef enum logic [1:0] {ORD, NVICT, VICT, NOT_USED } vict_labels;


    logic [12:0] cache_addr_hold;
    logic [2:0] funct3_hold;
    logic [5:0] addr_hold;
    logic [5:0] id_hold;
    logic ld_st_hold;
    logic [31:0] write_val_hold;

    logic front_valid, broad_hold, alr_broad;
    logic [4:0] mem_cnt;
    logic [1:0] next_next_vict;
    logic [12:0] cache_addr;
    logic miss;
    logic hit;
    logic [3:0] found;
    logic [tag_len:0] tag;
    logic [idx_len:0] idx, hold_idx;
    logic [3:0] hold_way;
    logic [5:0] byte_addr;
    assign tag = addr[31:(31-tag_len)];
    assign idx = addr[(1+ byte_bits) +: idx_len];
    assign byte_addr = addr[5:0];
    always_ff @(posedge clk or negedge reset)begin
        if(~reset)begin
            way1_valid <= 0;
            way2_valid <= 0;
            way3_valid <= 0;
            way4_valid <= 0;
            broad_hold <= 0;
            alr_broad <= 0;
            front_pointer <= 0;
            tail_pointer <= 0;
            trig_mem_read <= 0;
            read_busy <= 0;
            mem_cnt <= 0;
            broadcast_true <= 0;
            broadcast_addr <=0;
            broadcast_id <= 0;
            write_cache <= 0;
            cache_addr_out <= 0;
            funct3 <= 0;
            mem_cont_read_addr <= 0;
            mem_cont_write_addr <= 0;
            write_cache_val <= 0;
            addr_hold <= 0;
            funct3_hold <= 0;
            id_hold <= 0;
            cache_addr_hold <= 0;
            ld_st_hold <= 0;
            stall_forward <= 0;
            write_val_hold <= 0;
            hold_idx <= 0;
            hold_way <= 0;

            for(int i = 0; i<=sq_len; i++)begin
                sq_data[i] <= 0;
                sq_addr[i] <= 0;
                sq_found[i] <= 0;
                sq_funct3[i] <= 0;
                sq_idx[i] <=0;
            end

            for(int i = 0; i <= way_len; i ++)begin
                way1_tag[i] <= 0;
                way2_tag[i] <= 0;
                way3_tag[i] <= 0;
                way4_tag[i] <= 0;

                way_vict[i] <= 4'b0001;
                way_next_vict[i] <= 4'b0010;

                way1_data_offset[i] <= 64*i;
                way2_data_offset[i] <= 64*i + 64*(way_len + 1);
                way3_data_offset[i] <= 64*i + 128*(way_len + 1);
                way4_data_offset[i] <= 64*i + 192*(way_len + 1);
            end


        end else begin
            /*
                Load op:
                    1. check in ld_st forward to see if the mem addr has been written to recently
                        if found forward to broadcast and done
                    2. pass to cache controller
                    3. if cache hit
                        load and broadcast value
                    4. if cache miss
                        hold current lw instruction 
                        can process more hits, if another miss occurs cache needs to pause ld_st forward
                        repalce victim with correct block from memory
                            send init info to mem controller and wait for done flag
                        broadcast value
                
                Store op:
                 1. add to forward queue
                 2. send to cache controller and mem controller
                    3. Cache: controller
                        3.1 if hit:
                            write to cache at correct adder
                        3.2 if miss:
                            hold current sw instruction
                            can process more hits of either lw or sw variet, if a miss occurs cache needs to pause ld_st_forward
                            replace victim with correct block from memory
                            once block is in cache write value to correct adder
                    4. Mem controller
                        4.1 queue sw instruction
                            once cache bus is free start writing instructions from queue
                            if queue is full pause ld_st_forward until queue has space again

            */              
            if(broad_hold)begin
                broad_hold <= 0;
                stall_forward <= 0;
                write_cache_val <= write_val;
                if(load_store)begin
                    broadcast_true <= 1;
                    broadcast_addr <= addr_in;
                    broadcast_id <= id_in;
                    cache_addr_out <= cache_addr;
                    funct3 <= funct3_in;
                    write_cache <= 0;
                end else begin
                    broadcast_true <= 0;
                    broadcast_addr <= addr_in;
                    broadcast_id <= id_in;
                    cache_addr_out <= cache_addr;
                    funct3 <= funct3_in;
                    // write_cache_val <= 
                    write_cache <= 1;

                end

            end if(read_busy && mem_trans_done)begin
                read_busy <= 0;
                // stall_forward <= 0;
                broad_hold <= 1;
                // broadcast_val <= cache_read;
                write_cache_val <= write_val_hold;
                if(ld_st_hold)begin
                    broadcast_true <= 1;
                    broadcast_addr <= addr_hold;
                    broadcast_id <= id_hold;
                    cache_addr_out <= cache_addr_hold;
                    funct3 <= funct3_hold;
                    write_cache <= 0;
                end else begin
                    broadcast_true <= 0;
                    broadcast_addr <= addr_hold;
                    broadcast_id <= id_hold;
                    cache_addr_out <= cache_addr_hold;
                    funct3 <= funct3_hold;
                    write_cache <= 1;

                end

                case(hold_way)

                    4'b0001: way1_valid[hold_idx] <= 1;
                    4'b0010: way2_valid[hold_idx] <= 1;
                    4'b0100: way3_valid[hold_idx] <= 1;
                    4'b1000: way4_valid[hold_idx] <= 1;
                    default: begin end

                endcase



            end else if(miss && new_inst && ~read_busy) begin
                /*
                    If inst is a read:
                        Store read instruction, tell Forwarding block not to send any more loads
                        Identify victim block, Identify target block in memory
                        Set block to not valid
                        Set block tag
                        update vict, next vict
                        Transfer data from memory to block in cache
                        Set block to valid
                        read data and broadcast
                        tell forwarding block to send more load instructions

                    
                    If inst is a write:
                        If miss is generated because the tag is not valid
                        Queue write and wait unitl block has been loaded into the cache
                        write store

                        If miss is generated because the tag is not in the cache, shift write out
                
                */


                read_busy <= 1;
                write_cache <= 0;
                broadcast_true <= 0;

                
                funct3_hold <= funct3_in;
                addr_hold <= addr_in;
                id_hold <= id_in;
                ld_st_hold <= load_store;
                way_vict[idx] <= way_next_vict[idx];
                way_next_vict[idx] <= (1 << next_next_vict);
                hold_idx <= idx;
                trig_mem_read <= 1;
                mem_cont_read_addr <= addr[31:6];
                write_val_hold <= write_val;
                hold_way <= way_vict[idx];
                case(way_vict[idx])
                    4'b0001: begin
                        way1_valid[idx] <= 0;
                        way1_tag[idx] <= tag;
                        mem_cont_write_addr <= way1_data_offset[idx];
                        cache_addr_hold = way1_data_offset[idx] + byte_addr;
                    end
                    4'b0010: begin
                        way2_valid[idx] <= 0;
                        way2_tag[idx] <= tag;
                        mem_cont_write_addr <= way2_data_offset[idx];
                        cache_addr_hold = way2_data_offset[idx] + byte_addr;
                    end
                    4'b0100: begin
                        way3_valid[idx] <= 0;
                        way3_tag[idx] <= tag;
                        mem_cont_write_addr <= way3_data_offset[idx];
                        cache_addr_hold = way3_data_offset[idx] + byte_addr;
                    end
                    4'b1000: begin
                        way4_valid[idx] <= 0;
                        way4_tag[idx] <= tag;
                        mem_cont_write_addr <= way4_data_offset[idx];
                        cache_addr_hold = way4_data_offset[idx] + byte_addr;
                    end
                    default: begin end
                endcase

            end else if (hit && new_inst)begin
                if(found == way_vict[idx])begin //vict has hit
                    way_vict[idx] <= way_next_vict[idx];
                    way_next_vict[idx] <= (1<<next_next_vict);
                end else if(found == way_next_vict[idx])begin //next_vict has hit
                    way_next_vict[idx] <= (1<<next_next_vict);
                end

                write_cache_val <= write_val;
                if(load_store)begin //read from cache
                    write_cache <= 0;
                    broadcast_true <= 1;
                    broadcast_addr <= addr_in;
                    broadcast_id <= id_in;
                    cache_addr_out <= cache_addr;
                end else begin // write to cache, already broadcast by store load forwarding
                    write_cache <= 1;
                    broadcast_true <= 0;
                    broadcast_addr <= 0;
                    broadcast_id <= 0;
                    cache_addr_out <= cache_addr;
                end 



            end else begin
                if(~stall_forward_comb && ~stall_forward)
                    broadcast_true <= 0;

            end
            
            if(read_busy && miss && new_inst)begin
                stall_forward <= 1;
                write_cache <= 0;

            end 

            if(trig_mem_read)begin
                trig_mem_read <= 0;
            end


        end

    end


        always_comb begin
            stall_forward_comb = ((read_busy && miss && new_inst) || stall_forward ) && ~broad_hold;
            cache_addr = 0;
            found[0] = (way1_tag[idx] == tag) && way1_valid[idx];
            found[1] = (way2_tag[idx] == tag) && way2_valid[idx];
            found[2] = (way3_tag[idx] == tag) && way3_valid[idx];
            found[3] = (way4_tag[idx] == tag) && way4_valid[idx];
            hit = 0;
            miss = 0;
            if(found != 0)begin
                case(found)
                    4'b0001: cache_addr = way1_data_offset[idx] + byte_addr;
                    4'b0010: cache_addr = way2_data_offset[idx] + byte_addr;
                    4'b0100: cache_addr = way3_data_offset[idx] + byte_addr;
                    4'b1000: cache_addr = way4_data_offset[idx] + byte_addr;
                    default: cache_addr = 0;
                endcase
                hit = 1;
            end else begin
                miss = 1;
                /*
                    store addr into circular FIFO queue
                    start memory load with replacement policy, mark way at indx to be replaced as not valid
                    once the memory in the first place in the queue processed, check to see if the next element is in the same block
                    if not start another memory load

                    schedule writes around this


                */
            end      

            front_valid = 0;
            front_valid = ((sq_found[front_pointer] == 4'b0001) && way1_valid[sq_idx[front_pointer]]) ||
                            ((sq_found[front_pointer] == 4'b0010) && way2_valid[sq_idx[front_pointer]]) ||
                            ((sq_found[front_pointer] == 4'b0100) && way3_valid[sq_idx[front_pointer]]) ||
                            ((sq_found[front_pointer] == 4'b1000) && way4_valid[sq_idx[front_pointer]]);
            
        
            next_next_vict = 0;
            for(int i = 0; i < 4; i++ )begin
                if(~way_vict[idx][1<<i] && ~way_next_vict[idx][1<<i])begin
                    next_next_vict = i;
                end
            end





        end



    endmodule



module store_fifo(input logic [31:0] new_addr,
                  input logic [31:0] new_data,
                  input logic [2:0] new_funct3,
                  input logic new_inst,
                  input logic send_line,
                  input logic read_block,
                  input logic clk,
                  input logic reset,
                  output logic full,
                  output logic empty,
                  output logic [31:0] addr_out,
                  output logic [31:0] data_out,
                  output logic [2:0] funct3_out);

    localparam int fifo_len = 63; //+1
    logic [66:0] line_in;
    logic [66:0] fifo [fifo_len:0];
    logic [5:0] front_pointer, tail_pointer;
    logic [6:0] held;


    assign line_in = {new_addr, new_data, new_funct3};


    always_ff @(posedge clk or negedge reset)begin
        if(~reset)begin
            front_pointer <= 0;
            tail_pointer <= 0;
            held <= 0;
            addr_out <= 0;
            data_out <= 0;
            funct3_out <= 0;
            for(int i = 0; i <= fifo_len; i++)begin
                fifo[i] <= 0;
            end
        end else begin
            if(new_inst && ~full)begin
                fifo[tail_pointer] <= line_in;
                tail_pointer ++;
            end
            if(send_line & ~read_block)begin

                {addr_out, data_out, funct3_out} <= fifo[front_pointer];
                front_pointer ++;
            end

            if(new_inst && ~full && send_line && ~read_block)begin
                held <= held;
            end else if(new_inst & ~full)begin
                held <= held + 1;

            end else if(send_line && ~read_block)begin
                held <= held -1;
            end


        end



    end

    always_comb begin
        full = 0;
        if(front_pointer == tail_pointer && held != 0)begin
            full = 1;
        end
        empty = 0;
        if(held == 0)begin
            empty = 1;
        end
    end

endmodule

import defs_pkg::*;
module mem_cont(input logic [31:0] fifo_data,
                input logic [31:0] fifo_addr,
                input logic [2:0] fifo_funct3,
                input logic fifo_valid,

                input logic [25:0] block_addr,
                input logic [12:0] cache_addr,
                input logic read_block,
                
                input logic reset,
                input logic clk,

                output logic [31:0] read_data,
                output logic [12:0] read_addr,
                output logic write_cache,
                output logic read_fifo,

                output logic read_done);
    
    localparam int mem_len = 16 * 1000;
    logic [31:0] mem [mem_len - 1:0];



    logic [5:0] read_count;
    logic [31:0] base_addr;
    logic busy;
    logic [4:0] write_byte_addr;
    logic [12:0] cache_base_addr;


    assign write_byte_addr = fifo_addr[1:0] << 3;

    always_ff @(posedge clk or negedge reset)begin
        if(~reset)begin
            read_count <= 0;
            busy <= 0;
            base_addr <= 0;
            read_done <= 0;
            write_cache <= 0;
            read_fifo <= 0;
            read_data <= 0;
            read_addr <= 0;
            cache_base_addr <= 0;

            for(int i = 0; i < mem_len; i++)begin
                mem[i] <= 0;
            end
        end else begin
            if(fifo_valid && ~busy && ~read_block)begin
                case(funct3_types'(fifo_funct3))
                    BYTE: mem[fifo_addr[31:2]][write_byte_addr +: 8] <= fifo_data[7:0];
                    HALF: mem[fifo_addr[31:2]][write_byte_addr +: 16] <= fifo_data[15:0];
                    WORD: mem[fifo_addr[31:2]] <= fifo_data;
                    default: begin end
                endcase
                read_fifo <= 1;
            end else begin
                read_fifo <= 0;
            end


            if(read_block && ~busy && ~read_done)begin
                read_count <= 0;
                busy <= 1;
                base_addr <= block_addr << 6;
                cache_base_addr <= cache_addr;

            end else if(busy && ~read_done)begin
                if(read_count < 16) begin
                    read_addr <= cache_base_addr + (read_count << 2);
                    read_data <= mem[base_addr + read_count];
                    write_cache <= 1;


                    read_count ++;
                end else begin
                    busy <= 0;
                    read_done <= 1;
                    write_cache <= 0;
                end
            end else begin
                read_done <= 0;
                write_cache <= 0;
            end





        end
        
    end


endmodule
