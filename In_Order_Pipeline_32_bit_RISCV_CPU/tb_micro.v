module tb_mirco;

reg clk, reset;
wire [31:0] dataaddr;
wire [31:0] data_wrote;
wire write_enable;
wire [31:0] instruction;
integer cycle_count = 0;
parameter MAXCYCLES = 100;
    //create micro module instance
    micro M0(.clk(clk), .reset(reset), .result(data_wrote[31:0]), .result_addr(dataaddr[31:0]), .result_active(write_enable),
               .instruction_out(instruction[31:0]));

    //pulse active low reset to move CPU to save start state
    initial begin
        reset = 1; #5 reset = 0; #22 reset = 1;
    end

    //run clock forever
    initial begin
        clk = 0;
        forever #10 clk = ~clk;
    end

    //print current instruction to terminal for debugging
    always @(posedge clk)begin
        cycle_count = cycle_count + 1;

        $display("%0.2d: %b", cycle_count, instruction);

        //if more than n number of cycles have been complete the simulation has failed and the CPU needs to be debugged
        //prevents neverending simulations
        if(cycle_count > MAXCYCLES)begin
            $display("SIMULATION FAILED: Exceeded max clock cycles (%0d)", MAXCYCLES);
            $stop;
        
        end

    end

    //checks for final value in being stored in the data memory, if it is the correct value and no invalid writes
    //to the data memory occur then the CPU design is vaild
    always @(negedge clk)begin
        if(write_enable)begin
            if(dataaddr === 84 & data_wrote === 71)begin
                $display("SIMULATION SUCCEEDED");
                $stop;
            end else if(dataaddr === 84) begin
                $display("SIMULATION FAILED 2 addr: %d, data: %d", dataaddr, data_wrote);
                $stop;
            end
        end
    end



endmodule