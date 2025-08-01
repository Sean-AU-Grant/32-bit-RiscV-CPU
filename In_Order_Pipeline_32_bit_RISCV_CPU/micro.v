module micro(input wire clk,     //Toplevel module, routes design inputs and outputs
           input wire reset,
           output wire [31:0] result,
           output wire [31:0] result_addr,
           output wire result_active,
           output wire [31:0] instruction_out);

wire [31:0] internal_result;


datapath D0(.clk(clk), .reset(reset), .dataaddr_out(result_addr), .data_wrote_out(internal_result),
            .write_enable_out(result_active), .instruction_output(instruction_out));

assign result = internal_result;




endmodule



//control module, interprets instruction and outputs CPU controls
module control(input wire [6:0] opcode,
               input wire [2:0] funct3,
               input wire [6:0] funct7,
               output reg [5:0] alu_funct,
               output reg [2:0] data_mem_extend,
               output reg [12:0] controls);




    //controls order
    //  reg file write, immediate select, alu_b select, Data mem write, output select, branch_select, branch, jump          
    // R-type: 1, X, 0, 0, 0, 0, 0, 0
    // I-type: 1, I, 1, 0, 0, 0, 0, 0
    // S-type: 0, I, 1, 1, 0, 0, 0, 0
    // B-type: 0, I, 0, 0, 0, B, 1, 0
    // J-type  0, I, 0, 0, 0, 0, 0, 1   
    // 5 types of immediates (incase I use U-type instructions)
    // I: 001, S: 010, B: 011, J: 100


    // Data Mem extend
    //000 default
    //write:
    //000: write byte
    //001: write half
    //010: write word
    //read:
    //000: read byte, MSB extend
    //001: read half, MSB extend
    //010: read word
    //011: read byte, zero extend
    //100: read half, zero extend



    // wire [10] funct37; 
    // always @(*)begin
    //     funct37 = {funct3, funct7}

    //     case(funct37):
    //         10'h000: alu_funct = //add
    //         10'h020: alu_funct = //sub
    //         10'h400: alu_funct = //XOR
    //         10'h600: alu_funct = //OR
    //         10'h700: alu_funct = //AND
    //         10'h100: alu_funct = //logical left shift
    //         10'h500: alu_funct = //logical right shift
    //         10'h520: alu_funct = //logical right shift artithmetic
    //         10'h200: alu_funct = //Set less than
    //         10'h300: alu_funct = //set less than, zero extends
        
    //     endcase





        // end

        // Branch mode
    // 000: do not branch
    // 001: branch if zero
    // 010: branch if not zero
    // 011: branch if less than
    // 100: branch if greater than equal
    // 101: branch if less than unsigned not used, same as above
    // 110: branch if greater than unsigned not used

    //0: 0
    //1: add
    //2: subrtact
    //3: and
    //4: or
    //5: xor
    //6: slt
    //7: sltu
    //8:lls
    //9:lrs
    //10: mulsu
    //11: mulu
    //12: mul_low
    //13: mul_high
    //14: ars
    //15: divide
    //16: divide U
    //17: remainder
    //18 remainder u



    always @(*)begin

        //Switch case for instruction handling, will need to be replaced by a single large switch case in the future    
        case(opcode)
            // R type
            7'b0110011: begin 
                 controls = 13'b1_000_0_0_00_000_0_0;//R type
                 data_mem_extend = 3'h0;
                 case(funct3)
                    3'h0: begin
                        case(funct7)
                                7'h00: alu_funct = 6'b000001; //add
                                7'h01: alu_funct = 6'h0c; //MUL LOW
                                7'h20: alu_funct = 6'b000010; //sub
                            default: alu_funct = 6'h00;
                        endcase
                    end
                    3'h1: begin
                        case(funct7)
                            7'h00: alu_funct = 6'b001000; //Shift left logical
                            7'h01: alu_funct = 6'h0d; //MUL HIGH
                            default: alu_funct = 6'h00;
                        endcase
                    end
                    3'h2: begin
                        case(funct7)
                            7'h00: alu_funct = 6'b000110;//Set less than
                            7'h01: alu_funct = 6'h0a; //MUL HIGH Signed * unsigned
                            default: alu_funct = 6'h00;
                        endcase
                    end
                    3'h3: begin
                        case(funct7)
                            7'h00: alu_funct = 6'b000111; //Set less than U
                            7'h01: alu_funct = 6'h0b; //MUL HIGH UNSIGNED
                            default: alu_funct = 6'h00;
                        endcase
                    end
                    3'h4: begin
                        case(funct7)
                            7'h00: alu_funct = 6'b000101; //XOR
                            7'h01: alu_funct = 6'h0f; // DIVIDE
                            default: alu_funct = 6'h00;
                        endcase
                    end
                    3'h5: begin
                        case(funct7)
                            7'h00: alu_funct = 6'b001001; //Shift right logical
                            7'h01: alu_funct = 6'b10; //DIVIDE unsigned
                            7'h20: alu_funct = 6'b001110;//shift right arithemetic
                            default: alu_funct = 6'h00;
                        endcase
                    end
                    3'h6: begin
                        case(funct7)
                            7'h00: alu_funct = 6'b000100; //OR
                            7'h01: alu_funct = 6'h11; //Remainder
                            default: alu_funct = 6'h00;
                        endcase
                    end
                    3'h7: begin
                        case(funct7)
                            7'h00: alu_funct = 6'b000011;//AND
                            7'h01: alu_funct = 6'h12; //Remainder unsigned
                            default: alu_funct = 6'h00;
                        endcase
                    end
                    default: alu_funct = 6'b000000;
                 endcase

            // I type
            end
            7'b010011: begin
                 controls = 13'b1_001_1_0_00_000_0_0;//I type
                 data_mem_extend = 3'h0;
                  case(funct3)
                    3'h0: begin
                        alu_funct = 6'b000001;//add I
                    end
                    3'h1: begin
                        case(funct7)
                            7'h00: alu_funct = 6'b001000; //Shift left logical I
                            default: alu_funct = 6'h00;
                        endcase
                    end
                    3'h2: begin
                        alu_funct = 6'b000110;//set less tahn IM
                    end
                    3'h3: begin
                        alu_funct = 6'b000111; //Set less than I U zero extends    
                    end
                    3'h4: begin
                        alu_funct = 6'b000101;//XOR I 
                    end
                    3'h5: begin
                        case(funct7)
                            7'h00: alu_funct = 6'b001001;//Shift right logical I 
                            7'h20: alu_funct =  6'b001110;//shift right arithemetic I 
                            default: alu_funct = 6'h00;
                        endcase
                    end
                    3'h6: begin
                        alu_funct = 6'b000100;//OR I 
                    end
                    3'h7: begin
                        alu_funct = 6'b000011;//AND I 
                    end
                    default: alu_funct = 6'h00;
                 endcase
            end
            // I type
            7'b0000011: begin
                controls = 13'b1_001_1_0_01_000_0_0;
                alu_funct = 6'b000001;// ADD
                case(funct3)
                    3'h0:
                        data_mem_extend = 3'h0;
                    3'h1:
                        data_mem_extend = 3'h1;
                    3'h2:
                        data_mem_extend = 3'h2;
                    3'h3:
                        data_mem_extend = 3'h3;   
                    3'h4:
                        data_mem_extend = 3'h0;
                    3'h5:
                        data_mem_extend = 3'h1;
                    3'h6:
                        data_mem_extend = 3'h0;

                    3'h7:
                        data_mem_extend = 3'h0;

                    default: data_mem_extend = 3'h0;
                 endcase
            end
            //S type
            7'b0100011: begin
                controls = 13'b0_010_1_1_00_000_0_0;//S
                alu_funct = 6'b000001;//ADD
                case(funct3)
                    3'h0:
                        data_mem_extend = 3'h0;
                    3'h1:
                        data_mem_extend = 3'h1;
                    3'h2:
                        data_mem_extend = 3'h2;

                    default: data_mem_extend = 3'b0;
                 endcase
            end
            //B type
            7'b1100011: begin
                // controls = 13'b0_000_0_0_00_1_0;//B
                data_mem_extend = 3'b0;
                case(funct3)
                    3'h0: begin
                        controls = 13'b0_011_0_0_00_001_1_0;//B
                        alu_funct = 6'b000010;//subtract
                    end
                    3'h1: begin
                        controls = 13'b0_011_0_0_00_010_1_0;//B
                        alu_funct = 6'b000010;//subtract
                    end
                    3'h4: begin
                        controls = 13'b0_011_0_0_00_011_1_0;//B
                        alu_funct = 6'b000010;//subtract 
                    end
                    3'h5: begin
                        controls = 13'b0_011_0_0_00_100_1_0;//B
                        alu_funct = 6'b000010;//subtract  
                    end
                    3'h6: begin
                        alu_funct = 6'b010100;
                        controls = 13'b0_011_0_0_00_011_1_0;
                    end
                    3'h7: begin
                        alu_funct = 6'b010100;
                        controls = 13'b0_011_0_0_00_100_1_0;
                    end
                    //potential add unsigned comparisons, but need to change alu to do that
                    default: begin
                        controls = 13'b0_011_0_0_00_001_1_0;//B
                        alu_funct = 6'b000010;//subtract
                    end

                 endcase
            end
            //J type
            7'b1101111: begin // J
                controls = 13'b1_100_0_0_10_000_0_1;
                alu_funct = 6'h00;
                data_mem_extend = 3'h0;
            end
            // 7'b0110111: controls <= 8'b1_000_0_0_0_0_0;//U
            default: begin
                 controls = 13'b0000000000000;
                 alu_funct = 6'h0;
                 data_mem_extend = 3'h0;
            end

        
        endcase




    end    
endmodule


//interconnect for the entire cpu, all modules are created in this module
module datapath(input wire clk,
                input wire reset,
                output wire [31:0] dataaddr_out,
                output wire [31:0] data_wrote_out,
                output wire write_enable_out,
                output wire [31:0] instruction_output);


    wire [2:0] branch_mode;
    wire jump, branch, borrow;
    wire [31:0] next_pc;
    wire [31:0] pc;
    wire [2:0] im_source;
    wire zero;
    wire [31:0] b_mux;
    wire [31:0] alu_out;
    wire overflow;
    wire write_enable;
    wire [1:0] out_select;
    wire [31:0] dataaddr;
    wire[31:0] data_wrote;

    wire pc_jb;
    wire [31:0] pc_plus_4F;
    wire [31:0] imm_pcE;
    wire [31:0] instructionF;
    wire [31:0] pcD;
    wire [31:0] extend_outD;

    wire [31:0] resultW;
    wire [4:0] ra3W;
    wire [31:0] rd1D;
    wire [31:0] rd2D;
    wire [31:0] pc_plus_4D;
    wire [9:0] controlD;
    wire [5:0] alu_functD;
    wire [2:0] data_mem_extendD;
    wire [31:0] instructionD;

    wire[31:0] rd1E;
    wire[31:0] rd2E;
    wire [4:0] ra3E;
    wire [5:0] alu_functE;
    wire [2:0] data_mem_extendE;
    wire [3:0] controlE;
    wire [31:0] pc_plus_4E;
    wire [31:0] pcE;
    wire [31:0] immExE;
    wire[31:0] alu_a;
    wire [31:0] alu_b_reg;
    wire alu_b_sel;
    wire [31:0] alu_resM;
    wire [31:0] write_dataM;
    wire [4:0] ra3M;
    wire [31:0] pc_plus_4M;
    wire [2:0] controlM;
    wire [2:0] data_mem_extendM;
    wire [31:0] read_dataM;
    wire [31:0] alu_resW;
    wire [31:0] read_dataW;
    wire [31:0] pc_plus_4W;
    wire pc_stall;
    wire decode_stall;
    wire decode_flush;
    wire execute_flush;
    wire [1:0] rd1_mux;
    wire [1:0] rd2_mux;
    wire we3W;
    
    wire [4:0] ra1E;
    wire [4:0] ra2E;

    assign write_enable_out = write_enable;
    assign dataaddr_out = dataaddr;
    assign data_wrote_out = data_wrote;
    assign instruction_output = instructionF;

    control C0 (.opcode(instructionD[6:0]), .funct3(instructionD[14:12]), .funct7(instructionD[31:25]), .alu_funct(alu_functD[5:0]),
                .data_mem_extend(data_mem_extendD[2:0]), 
                .controls({controlD[9], im_source[2:0], controlD[8:0]}));

    pc_br_jm PC4(.branch_mode(branch_mode[2:0]), .branch(branch), .jump(jump), .zero(zero), .borrow(borrow), .pc_jb(pc_jb));

    
    pc PC0(.clk(clk), .reset(reset), .enable(pc_stall), .new_pc(next_pc[31:0]), .pc(pc[31:0]));

    pc_mux PC1(.pc_plus_4(pc_plus_4F[31:0]), .imm_pc(imm_pcE[31:0]), .pc_jb(pc_jb), .pc(next_pc[31:0]));

    pc_plus_4 PC2(.pc(pc[31:0]), .new_pc(pc_plus_4F[31:0]));

    pc_target PC3(.pc(pcE[31:0]), .jump_val(immExE[31:0]), .target(imm_pcE[31:0]));

    i_mem I0(.a(pc[31:0]), .read_data(instructionF[31:0]));

    decode_reg DR(.instructionF(instructionF[31:0]), .pcF(pc[31:0]), .pc_plus_4F(pc_plus_4F[31:0]), .enable(decode_stall), 
                  .clk(clk), .clr(decode_flush), .reset(reset), .instructionD(instructionD[31:0]), .pc_plus_4D(pc_plus_4D[31:0]), 
                  .pcD(pcD[31:0]));


    i_extender E0(.in(instructionD[31:7]), .im_source(im_source[2:0]), .ex_im(extend_outD[31:0]));

    register_file R0(.a1(instructionD[19:15]), .a2(instructionD[24:20]), .wa3(ra3W[4:0]), .wd3(resultW[31:0]), .clk(clk),
                     .we3(we3W), .reset(reset), .rd1(rd1D[31:0]), .rd2(rd2D[31:0]));

    execute_reg ER(.rd1D(rd1D[31:0]), .rd2D(rd2D[31:0]), .ra1D(instructionD[19:15]), .ra2D(instructionD[24:20]), .ra3D(instructionD[11:7]), .pcD(pcD[31:0]), .immExD(extend_outD[31:0]), 
                   .pc_plus_4D(pc_plus_4D[31:0]), .controlD(controlD[9:0]), .alu_functD(alu_functD[5:0]), .data_mem_extendD(data_mem_extendD[2:0]),
                   .clr(execute_flush), .clk(clk), .reset(reset), .rd1E(rd1E[31:0]), .rd2E(rd2E[31:0]), .ra1E(ra1E), .ra2E(ra2E), .ra3E(ra3E[4:0]), .pcE(pcE[31:0]), 
                   .immExE(immExE[31:0]), .pc_plus_4E(pc_plus_4E[31:0]), .controlE({controlE[3], alu_b_sel, controlE[2:0], branch_mode[2:0], branch, jump}), .alu_functE(alu_functE[5:0]),
                   .data_mem_extendE(data_mem_extendE[2:0]));


    r1_forward F1(.reg1(rd1E[31:0]), .alu_resM(alu_resM[31:0]), .resultW(resultW[31:0]), .select(rd1_mux[1:0]), .alu_a(alu_a[31:0]));

    r2_forward F2(.reg2(rd2E[31:0]), .alu_resM(alu_resM[31:0]), .resultW(resultW[31:0]), .select(rd2_mux[1:0]), .alu_b(alu_b_reg[31:0]));

    alu A0(.a(alu_a[31:0]), .b(b_mux[31:0]), .op(alu_functE[5:0]), .out(alu_out[31:0]), .zero(zero), .borrow(borrow), .overflow(overflow));

    alu_b_mux A1(.rd2(alu_b_reg[31:0]), .immediate(immExE[31:0]), .select(alu_b_sel), .alu_b(b_mux[31:0]));


    memory_reg MR(.alu_resE(alu_out[31:0]), .write_dataE(alu_b_reg[31:0]), .ra3E(ra3E[4:0]), .pc_plus_4E(pc_plus_4E[31:0]), 
                  .controlE(controlE[3:0]), .data_mem_extendE(data_mem_extendE[2:0]), .clk(clk), .reset(reset), .alu_resM(alu_resM[31:0]), 
                  .write_dataM(write_dataM[31:0]), .ra3M(ra3M[4:0]), .pc_plus_4M(pc_plus_4M[31:0]), .controlM({controlM[2], write_enable, controlM[1:0]}), 
                  .data_mem_extendM(data_mem_extendM[2:0]));
    
    d_mem D0(.clk(clk), .reset(reset), .write_enable(write_enable), .a(alu_resM[31:0]), .write_data(write_dataM[31:0]), 
             .extend(data_mem_extendM[2:0]), .dataaddr(dataaddr[31:0]), .data_wrote(data_wrote[31:0]), .read_data(read_dataM[31:0]));




    write_reg WR(.alu_resM(alu_resM[31:0]), .read_dataM(read_dataM[31:0]), .pc_plus_4M(pc_plus_4M[31:0]),
                 .ra3M(ra3M[4:0]), .controlM(controlM[2:0]), .clk(clk), .reset(reset), .alu_resW(alu_resW[31:0]),
                 .read_dataW(read_dataW[31:0]), .pc_plus_4W(pc_plus_4W[31:0]), .ra3W(ra3W[4:0]), .controlW({we3W,out_select[1:0]}));

    out_select O0(.alu_src(alu_resW[31:0]), .d_mem_src(read_dataW[31:0]), .pc_src(pc_plus_4W[31:0]), .select(out_select[1:0]),
                  .data_out(resultW[31:0]));
    
    hazard_control H0(.ra1D(instructionD[19:15]), .ra2D(instructionD[24:20]), .ra1E(ra1E), .ra2E(ra2E), 
                      .we3M(controlM[2]), .we3W(we3W), .ra3E(ra3E[4:0]), .ra3M(ra3M[4:0]), .ra3W(ra3W[4:0]), .out_select(out_select[1:0]),
                      .pc_jb(pc_jb), .pc_stall(pc_stall), .decode_stall(decode_stall), .execute_flush(execute_flush), 
                      .decode_flush(decode_flush), .rd1_mux(rd1_mux[1:0]), .rd2_mux(rd2_mux[1:0]));





endmodule


//REGISTERS F,D,E,M,W
module decode_reg(input wire[31:0] instructionF,
                  input wire [31:0] pcF,
                  input wire [31:0] pc_plus_4F,
                  input wire enable,
                  input wire clk,
                  input wire clr,
                  input wire reset,
                  output reg [31:0] instructionD,
                  output reg [31:0] pc_plus_4D,
                  output reg [31:0] pcD);
        //controls data shift from input to output
    always @(posedge clk or negedge reset) begin
        if (~reset) begin // Asynchronous active-low reset
            instructionD <= 0;
            pcD <= 0;
            pc_plus_4D <= 0;
        end else if (clr) begin // Synchronous clear/flush
            instructionD <= 0;
            pcD <= 0;
            pc_plus_4D <= 0;
        end else if (~enable) begin // Synchronous enable for normal operation
            instructionD <= instructionF;
            pcD <= pcF;
            pc_plus_4D <= pc_plus_4F;
        end else begin 
            instructionD <= instructionD;
            pcD <= pcD;
            pc_plus_4D <= pc_plus_4D;
        end
    end

endmodule



module execute_reg(input wire [31:0] rd1D,
                   input wire [31:0] rd2D,
                   input wire [4:0] ra1D,
                   input wire [4:0] ra2D,
                   input wire [4:0] ra3D,
                   input wire [31:0] pcD,
                   input wire [31:0] immExD,
                   input wire [31:0] pc_plus_4D,
                   input wire [9:0] controlD,
                   input wire [5:0] alu_functD,
                   input wire [2:0] data_mem_extendD,
                   input wire clr,
                   input wire clk,
                   input wire reset,
                   output reg [31:0] rd1E,
                   output reg [31:0] rd2E,
                   output reg [4:0] ra1E,
                   output reg [4:0] ra2E,
                   output reg [4:0] ra3E,
                   output reg [31:0] pcE,
                   output reg [31:0] immExE,
                   output reg [31:0] pc_plus_4E,
                   output reg [9:0] controlE,
                   output reg [5:0] alu_functE,
                   output reg [2:0] data_mem_extendE);

always @(posedge clk or negedge reset) begin // Correct sensitivity list for asynchronous reset
    if (~reset) begin // Asynchronous active-low reset - takes highest precedence
        rd1E <= 0;
        rd2E <= 0;
        ra3E <= 0;
        pcE <= 0;
        immExE <= 0;
        pc_plus_4E <= 0;
        controlE <= 0;
        alu_functE <= 0;
        ra1E <= 0;
        ra2E <= 0;
        data_mem_extendE <= 0;
    end else if (clr) begin // Synchronous clear/flush - happens on clock edge, only if not in async reset
        rd1E <= 0; // Clear to 0 when clr is active
        rd2E <= 0;
        ra3E <= 0;
        pcE <= 0;
        immExE <= 0;
        pc_plus_4E <= 0;
        controlE <= 0;
        alu_functE <= 0;
        ra1E <= 0;
        ra2E <= 0;
        data_mem_extendE <= 0;
    end else begin // Normal operation - happens on clock edge, only if not in reset or clear
        rd1E <= rd1D;
        rd2E <= rd2D;
        ra3E <= ra3D;
        pcE <= pcD;
        immExE <= immExD;
        pc_plus_4E <= pc_plus_4D;
        controlE <= controlD;
        alu_functE <= alu_functD;
        data_mem_extendE <= data_mem_extendD;
        ra1E <= ra1D;
        ra2E <= ra2D;
    end
end

endmodule



module memory_reg(input wire [31:0] alu_resE,
                  input wire [31:0] write_dataE,
                  input wire [4:0] ra3E,
                  input wire [31:0] pc_plus_4E,
                  input wire [3:0] controlE,
                  input wire [2:0] data_mem_extendE, 
                  input wire clk,
                  input wire reset,
                  output reg [31:0] alu_resM,
                  output reg [31:0] write_dataM,
                  output reg [4:0] ra3M,
                  output reg [31:0] pc_plus_4M,
                  output reg [3:0] controlM,
                  output reg [2:0] data_mem_extendM);
    always @(posedge clk or negedge reset)begin
        if(~reset)begin
            alu_resM <= 0;
            write_dataM <= 0;
            ra3M <= 0;
            pc_plus_4M <= 0;
            controlM <= 0;
            data_mem_extendM <= 0;
        end else begin
            alu_resM <= alu_resE;
            write_dataM <= write_dataE;
            ra3M <= ra3E;
            pc_plus_4M <= pc_plus_4E;
            controlM <= controlE;
            data_mem_extendM <= data_mem_extendE;
        end
    end
endmodule





module write_reg(input wire [31:0] alu_resM,
                 input wire [31:0] read_dataM,
                 input wire [31:0] pc_plus_4M,
                 input wire [4:0] ra3M,
                 input wire [2:0] controlM,
                 input wire clk,
                 input wire reset,
                 output reg [31:0] alu_resW,
                 output reg [31:0] read_dataW,
                 output reg [31:0] pc_plus_4W,
                 output reg [4:0] ra3W,
                 output reg [2:0] controlW);
    always @(posedge clk or negedge reset )begin
        if(~reset)begin
            alu_resW <= 0;
            read_dataW <= 0;
            pc_plus_4W <= 0;
            ra3W <= 0;
            controlW <= controlM;
        end else begin
            alu_resW <= alu_resM;
            read_dataW <= read_dataM;
            pc_plus_4W <= pc_plus_4M;
            ra3W <= ra3M;
            controlW <= controlM;
        end
    end
endmodule


//Fowarding muxes, 00 outputs zero so cpu reset enters a stable state
module r1_forward(input wire [31:0] reg1,
                  input wire [31:0] alu_resM,
                  input wire [31:0] resultW,
                  input wire [1:0] select,
                  output reg [31:0] alu_a);
    always @(*)begin
        case(select)
            2'b00: alu_a <= 32'b0;
            2'b01: alu_a <= reg1;
            2'b10: alu_a <= alu_resM;
            2'b11: alu_a <= resultW;
            default: alu_a <= 32'b0;
        endcase
    end

endmodule

module r2_forward(input wire [31:0] reg2,
                  input wire [31:0] alu_resM,
                  input wire [31:0] resultW,
                  input wire [1:0] select,
                  output reg [31:0] alu_b);
    always @(*)begin
        case(select)
            2'b00: alu_b <= 32'b0;
            2'b01: alu_b <= reg2;
            2'b10: alu_b <= alu_resM;
            2'b11: alu_b <= resultW;
            default: alu_b <= 32'b0;
        endcase
    end

endmodule


//hazard control, combinationally monitors for any possible hazards and executes controls as need to avoid them
module hazard_control(input wire [4:0] ra1D,
                      input wire [4:0] ra2D,
                      input wire [4:0] ra1E,
                      input wire [4:0] ra2E,
                      input wire we3M,
                      input wire we3W,
                      input wire [4:0] ra3E,
                      input wire [4:0] ra3M,
                      input wire [4:0] ra3W,
                      input wire [1:0] out_select,
                      input wire pc_jb,
                      output wire pc_stall,
                      output wire decode_stall,
                      output wire execute_flush,
                      output wire decode_flush, 
                      output reg [1:0] rd1_mux,
                      output reg [1:0] rd2_mux);
    //forward equation: (rd1/2E == ra3M) && WE3M
    //                  (rd1/2E == ra3W) && WE3W
    //                  (else normal)
    //stall equation: lw & (rd1/2D == RA3E)
    //J/B   equation: pc_jb -> flush execute and decode
    // wire forward_M_1, forward_W_1;

    // wire ra_match = (ra1D == ra3E); // This is a 5-bit comparator, potentially many gates
    // wire out_select_is_01 = (out_select == 2'b01); // Simple 2-bit comparison

    // wire load_use_hazard = ra_match & out_select_is_01;
    wire lw_stall;
    wire r1_match;
    wire r2_match;
    wire result_src;
    
    assign r1_match = (ra1D == ra3E) ? 1:0;
    assign r2_match = (ra2D == ra3E) ? 1:0;
    assign result_src = (out_select == 2'b01) ? 1:0;
    assign lw_stall = (r1_match | r2_match) & result_src ? 1 : 0;

    assign decode_flush = pc_jb;
    assign pc_stall = lw_stall;
    assign decode_stall = lw_stall;
    assign execute_flush = pc_jb | lw_stall;

    always @(*)begin
        if((ra1E == ra3M) && we3M)begin
            rd1_mux = 2'b10;

        end else if((ra1E == ra3W) && we3W)begin
            rd1_mux = 2'b11;

        end else begin
            rd1_mux = 2'b01;
        end




        if((ra2E == ra3M) && we3M)begin
            
            rd2_mux = 2'b10;


        end else if((ra2E == ra3W) && we3W)begin
            rd2_mux = 2'b11;


        end else begin
            rd2_mux = 2'b01;

        end

    end



endmodule


//controls if rd2 or the immediate is sent to the alu b input
module alu_b_mux(input wire [31:0] rd2,
                 input wire [31:0] immediate,
                 input wire select,
                 output wire [31:0] alu_b);
    
    assign alu_b = select ? immediate : rd2;
endmodule

//logic to determine if the pc needs to branch, output goes to pc input mux
module pc_br_jm(input wire [2:0] branch_mode,
                input wire branch,
                input wire jump,
                input wire zero,
                input wire borrow,
                output reg pc_jb);
    // Branch mode
    // 000: do not branch
    // 001: branch if zero
    // 010: branch if not zero
    // 011: branch if less than
    // 100: branch if greater than equal
    // 101: branch if less than unsigned not used, same as above
    // 110: branch if greater than unsigned not used
    reg should_branch; 
    always @(*)begin
        if(branch)begin
            case(branch_mode)
                3'h0: begin
                    should_branch = 1'b0;
                end
                3'h1:begin
                    should_branch = (zero)? 1'b1 : 1'b0;
                end
                3'h2:begin
                    should_branch = (~zero) ? 1'b1: 1'b0;
                end
                3'h3:begin
                    should_branch = borrow ? 1'b1: 1'b0;
                end
                3'h4:begin
                    should_branch = ~borrow ? 1'b1: 1'b0;
                    
                end
                default: should_branch = 1'b0;
            endcase
        end else should_branch = 1'b0;
        
        pc_jb = should_branch | jump;
    end

endmodule



//determines which value is sent to the pc to be shifted in next
module pc_mux(input wire [31:0] pc_plus_4,
              input wire [31:0] imm_pc,
              input wire pc_jb,
              output wire [31:0] pc);
     assign  pc = pc_jb ? imm_pc : pc_plus_4; 
endmodule


//output mux, a 3:1 mux that determines the output of the CPU in the write back stage
module out_select(input wire [31:0] alu_src,
                  input wire [31:0] d_mem_src,
                  input wire [31:0] pc_src,
                  input wire [1:0] select,
                  output reg [31:0] data_out);


    always @(*)begin
        case(select)
            2'h0: data_out <= alu_src;
            2'h1: data_out <= d_mem_src;
            2'h2: data_out <= pc_src;
            default: data_out <= alu_src;
        endcase

    end
endmodule


//Immediate extender, decodes instruction bits 31:8 into a 32-bit value
module i_extender(input wire [24:0] in,
                  input wire [2:0] im_source,
                  output reg [31:0] ex_im);

    always @(*)begin
        case(im_source)
            3'b000: ex_im <= 0;
            3'b001: ex_im <= {{20{in[24]}},in[24:13]}; //I type:
            3'b010: ex_im <= {{20{in[24]}},in[24:18], in[4:0]}; //S type
            3'b011: ex_im <= {{20{in[24]}},in[24], in[0], in[23: 19], in[4:1], 1'b0}; //B type
            3'b100: ex_im <= {{11{in[24]}}, in[24], in[12:5], in[13], in[23:14], 1'b0}; //J type
            3'b101: ex_im <= {in[24: 6] , 12'b000000000000}; //U type
            3'b110: ex_im <= 0;
            3'b111: ex_im <= 0;
            default: ex_im <= 0;

        endcase
    end
endmodule


//adds the immediate to the current pc for jumps and branches
module pc_target(input wire [31:0] pc,
                 input wire [31:0] jump_val,
                 output reg [31:0] target);

    always @(*)begin
        target = pc + jump_val;
    end
endmodule


//PC DFF, with reset and stall functionality
module pc(input wire clk,
          input wire reset,
          input wire enable,
          input wire [31:0] new_pc,
          output reg [31:0] pc);

    always @(posedge clk or negedge reset)begin
        if (~reset)
            pc <= 0;
        else if(~enable)
            pc <= new_pc;
        else
            pc<=pc;


    end
endmodule


//adds 4 to the pc to change the pc to the next instruction address
module pc_plus_4(input wire [31:0] pc,
                 output reg [31:0] new_pc); 
                 
    always @(*)begin
        new_pc = pc+4;
    end                              
endmodule


//Register data for 32 32-bit registers
module register_file(input wire [4:0] a1,
                     input wire [4:0] a2,
                     input wire [4:0] wa3,
                     input wire [31:0] wd3,
                     input wire clk,
                     input wire we3,
                     input wire reset,
                     output reg [31:0] rd1,
                     output reg [31:0] rd2);
    
    reg [31:0] registers [31:0];
    
    integer i;
 

    always @(posedge clk or negedge reset) begin
        if(~reset)begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'b0;
            end
        end else if (we3) begin
            registers[wa3] <= wd3;
        end

    end

    always @(negedge clk)begin
        rd1 <= (a1 == wa3 && we3 && a1 != 0) ? wd3 : registers[a1];
        rd2 <= (a2 == wa3 && we3 && a2 != 0) ? wd3 : registers[a2];

    end
endmodule


//data memory, 64 32-bit words of memory, includes half word, byte, and word read and write operations
module d_mem(input wire clk,
             input wire reset,
             input wire write_enable,
             input wire [31:0] a, 
             input wire [31:0] write_data,
             input wire [2:0] extend,
             output reg [31:0] dataaddr,
             output reg [31:0] data_wrote,
             output reg [31:0] read_data);

    reg [31:0] RAM[63:0];

    reg [7:0] byte;
    reg [15:0] half_word;
    reg [32:0] word;

    integer j;
    // initial begin
    //     for (j = 0; j < 64; j = j + 1) begin
    //         RAM[j] = 32'b0;
    //     end
    // end

    //000 default
    //write:
    //000: write byte
    //001: write half
    //010: write word
    //read:
    //000: read byte, MSB extend
    //001: read half, MSB extend
    //010: read word
    //011: read byte, zero extend
    //100: read half, zero extend
    always @(*)begin
        data_wrote = write_data;
        dataaddr = a;
        word = RAM[a[7:2]];

        case(a[1:0])
            2'h0:begin
                byte = RAM[a[7:2]][7:0];
                half_word = RAM[a[7:2]][15:0];
            end
            2'h1:begin
                byte = RAM[a[7:2]][15:8];
                half_word = RAM[a[7:2]][15:0];
            end
            2'h2:begin
                byte = RAM[a[7:2]][23:16];
                half_word = RAM[a[7:2]][31:16];

            end
            2'h3: begin
                byte = RAM[a[7:2]][31:24];
                half_word = RAM[a[7:2]][31:16];

            end
            default:begin
                byte = 8'b0;
                half_word = 16'b0;
            end
        endcase




        case(extend)
            3'h0: read_data = {{24{byte[7]}}, byte};// load byte
            3'h1: read_data = {{16{half_word[15]}}, half_word}; //load word
            3'h2: read_data = word;
            3'h3: read_data = {24'h000, byte};
            3'h4: read_data = {16'h00, half_word};
            default: read_data = word;
        endcase

    end

    always @(posedge clk or negedge reset)begin 
        if(~reset)begin
            for (j = 0; j < 64; j = j + 1) begin
                RAM[j] <= 32'b0;
            end
        end else if(write_enable) begin

            case(extend)
                3'b000: RAM[a[7:2]][7:0]  <= write_data[7:0]; //store byte
                3'b001: RAM[a[7:2]][15:0]  <= write_data[15:0]; // store half word
                3'b010: RAM[a[7:2]]  <= write_data; //store word
                default: RAM[a[7:2]]  <= write_data;
            endcase
        end 
    end
endmodule


//instruction memory, currently hardcoded for synthesis but will be modified in the future to be programmable
module i_mem(
    input wire [31:0] a, 
    output reg [31:0] read_data);

    // reg [31:0] RAM[0:63];  // 64 words of 32-bit instruction memory

    always @(*)begin
      case(a[7:2])
        6'h00: read_data = 32'h00500093;
        6'h01: read_data = 32'h00a00113;
        6'h02: read_data = 32'h05400193;
        6'h03: read_data = 32'h00300213;
        6'h04: read_data = 32'h00700293;
        6'h05: read_data = 32'h00208333;
        6'h06: read_data = 32'h401303b3;
        6'h07: read_data = 32'h0013c433;
        6'h08: read_data = 32'h0013e4b3;
        6'h09: read_data = 32'h0084f533;
        6'h0A: read_data = 32'h005215b3;
        6'h0B: read_data = 32'h0055d633;
        6'h0C: read_data = 32'h4055d6b3;
        6'h0D: read_data = 32'h00522733;
        6'h0E: read_data = 32'h0042b7b3;
        6'h0F: read_data = 32'hffb10813;
        6'h10: read_data = 32'h00714893;
        6'h11: read_data = 32'h0f016913;
        6'h12: read_data = 32'h00f97993;
        6'h13: read_data = 32'h00221a13;
        6'h14: read_data = 32'h002a5a93;
        6'h15: read_data = 32'h402a5b13;
        6'h16: read_data = 32'h01422b93;
        6'h17: read_data = 32'h0032bc13;
        6'h18: read_data = 32'h00252023;
        6'h19: read_data = 32'h00151223;
        6'h1A: read_data = 32'h00150423;
        6'h1B: read_data = 32'h00052c83;
        6'h1C: read_data = 32'h00451d03;
        6'h1D: read_data = 32'h00850d83;
        6'h1E: read_data = 32'h00455e03;
        6'h1F: read_data = 32'h00854e83;
        6'h20: read_data = 32'h01ac8f33;
        6'h21: read_data = 32'h00208663;
        6'h22: read_data = 32'h00209c63;
        6'h23: read_data = 32'h0021a023;
        6'h24: read_data = 32'h0020cc63;
        6'h25: read_data = 32'h00115c63;
        6'h26: read_data = 32'h0020e863;
        6'h27: read_data = 32'h00117863;
        6'h28: read_data = 32'h00100f93;
        6'h29: read_data = 32'h0080036f;
        6'h2A: read_data = 32'h00200f93;
        6'h2B: read_data = 32'h00300f93;
        6'h2C: read_data = 32'h00400393;
        6'h2D: read_data = 32'h00038467;
        6'h2E: read_data = 32'h04600493;
        6'h2F: read_data = 32'h00148493;
        6'h30: read_data = 32'h0091a023;
        default: read_data = 32'b0;
    endcase


    end
    // integer i;
    // initial begin
    //     // $readmemh("test_prg.hex", read_data);
    //     RAM[0] = 32'h00500113;
    //     RAM[1] = 32'h00c00193;
    //     RAM[2] = 32'hff718393;
    //     RAM[3] = 32'h0023e233;
    //     RAM[4] = 32'h0041f2b3;
    //     RAM[5] = 32'h004282b3;
    //     RAM[6] = 32'h02728663;
    //     RAM[7] = 32'h0041a233;
    //     RAM[8] = 32'h00920463;
    //     RAM[9] = 32'h00000293;
    //     RAM[10] = 32'h0023a233;
    //     RAM[11] = 32'h005203b3;
    //     RAM[12] = 32'h402383b3;
    //     RAM[13] = 32'h0471a223;
    //     RAM[14] = 32'h05002103;
    //     RAM[15] = 32'h008001ef;
    //     RAM[16] = 32'h00100113;
    //     RAM[17] = 32'h00310133;
    //     RAM[18] = 32'h04202a23;

    //     for(i = 19; i<64; i = i+1)begin
    //         RAM[i] = 32'b0;
    //     end
    // end



    // assign read_data = RAM[a[7:2]];  // Use word addressing (not byte indexing)
endmodule


//alu toplevel module, inclues extra opcodes for future developments and additions
module alu(input wire [31:0] a,
           input wire [31:0] b,
           input wire [5:0] op, //64 possible op codes
           output wire [31:0] out,
           output wire zero,
           output wire borrow,
           output wire overflow);
    wire borrows, borrowu;
    wire [31:0] add, sub, and_wire, or_wire, xor_wire, slt, sltu, lls, lrs, ars, 
                mul_low, mul_high, mulsu, mulu, div, divu, rem, remu, subu;
    wire carry;
    adder A0(.a(a[31:0]), .b(b[31:0]), .cin({1'b0}), .sum(add[31:0]), .carry(carry));
    subtract S0(.a(a[31:0]), .b(b[31:0]), .diff(sub[31:0]), .borrow(borrow));
    logical_left_shift L0(.a(a[31:0]), .b(b[31:0]), .shifted(lls[31:0]));
    logical_right_shift R0(.a(a[31:0]), .b(b[31:0]), .shifted(lrs[31:0]));
    arithmetic_right_shift R1(.a(a[31:0]), .b(b[31:0]), .shifted(ars[31:0]));
    set_less_than SL0(.a(a[31:0]), .b(b[31:0]), .result(slt[31:0]));
    set_less_than_u SL1(.a(a[31:0]), .b(b[31:0]), .result(sltu[31:0]));


    genvar i;
    generate
        for(i = 0; i < 32; i = i+1)begin: logic
            and(and_wire[i], a[i], b[i]);
            or(or_wire[i], a[i], b[i]);
            xor(xor_wire[i], a[i], b[i]);
        end
        
    endgenerate

    //0: 0
    //1: add
    //2: subrtact
    //3: and
    //4: or
    //5: xor
    //6: slt
    //7: sltu
    //8:lls
    //9:lrs
    //10: mulsu
    //11: mulu
    //12: mul_low
    //13: mul_high
    //14: ars
    //15: divide
    //16: divide U
    //17: remainder
    //18 remainder u
    //19: subtract u

    //all multiply, divide, and remainder instructions are not supported currently

    alu_mux MX(.out_1(add[31:0]), .out_2(sub[31:0]), .out_3(and_wire[31:0]), .out_4(or_wire[31:0]), .out_5(xor_wire[31:0]),
            .out_6(slt[31:0]), .out_7(sltu[31:0]), .out_8(lls[31:0]), .out_9(lrs[31:0]), .out_10({32'b0}), .out_11({32'b0}),
            .out_12({32'b0}), .out_13({32'b0}), .out_14(ars[31:0]), .out_15({32'b0}), .out_16({32'b0}), .out_17({32'b0}),
            .out_18({32'b0}), .out_19({32'b0}), .op(op[5:0]), .out_wire(out[31:0]));



    assign zero = (out == 32'b0);
endmodule

//ALU output mux, controls what operation is output based on the control
module alu_mux(input wire [31:0] out_1,
               input wire [31:0] out_2,
               input wire [31:0] out_3,
               input wire [31:0] out_4,
               input wire [31:0] out_5,
               input wire [31:0] out_6,
               input wire [31:0] out_7,
               input wire [31:0] out_8,
               input wire [31:0] out_9,
               input wire [31:0] out_10,
               input wire [31:0] out_11,
               input wire [31:0] out_12,
               input wire [31:0] out_13,
               input wire [31:0] out_14,
               input wire [31:0] out_15,               
               input wire [31:0] out_16,               
               input wire [31:0] out_17,               
               input wire [31:0] out_18,
               input wire [31:0] out_19,              
               input wire [5:0] op,
               output reg [31:0] out_wire);

    always @(*)begin
    
        case(op)
            6'b000000: out_wire = 32'h0000;
            6'b000001: out_wire = out_1;
            6'b000010: out_wire = out_2;
            6'b000011: out_wire = out_3;
            6'b000100: out_wire = out_4;
            6'b000101: out_wire = out_5;
            6'b000110: out_wire = out_6;
            6'b000111: out_wire = out_7;
            6'b001000: out_wire = out_8;
            6'b001001: out_wire = out_9;
            6'b001010: out_wire = out_10;
            6'b001011: out_wire = out_11;
            6'b001100: out_wire = out_12;
            6'b001101: out_wire = out_13;
            6'b001110: out_wire = out_14;
            6'b001111: out_wire = out_15;
            6'b010000: out_wire = out_16;
            6'b010001: out_wire = out_17;
            6'b010011: out_wire = out_18;
            6'b010100: out_wire = out_19;
            default: out_wire = 32'h0000;
        endcase
    
    
    end
endmodule


//operation modules for all ALU operations, future development in this area would be optimizing the logic for speed
// i.e using a carry look-ahead adder etc to minimize delay through this critical path

module set_less_than(input wire signed [31:0] a,
                     input wire signed [31:0] b,
                     output wire [31:0] result);
    assign result = (a < b) ? 1 : 0;
endmodule

module set_less_than_u(input wire [31:0] a,
                     input wire  [31:0] b,
                     output wire [31:0] result);
    assign result = (a < b) ? 1 : 0;
endmodule


module logical_left_shift(input wire [31:0] a,
                     input wire [31:0] b,
                     output wire [31:0] shifted);
    assign shifted = a << b;
endmodule

module logical_right_shift(input wire [31:0] a,
                           input wire [31:0] b,
                           output wire [31:0] shifted);

    assign shifted = a >> b;
endmodule

module arithmetic_right_shift(input wire signed [31:0] a,
                           input wire [31:0] b,
                           output wire signed [31:0] shifted);

    assign shifted = a >>> b;
endmodule


module subtract(input wire[31:0] a,
                input wire [31:0] b, 
                output wire [31:0] diff,
                output wire borrow);

    assign {borrow, diff} = {1'b0, a} - {1'b0, b};
endmodule


module adder(input wire [31:0] a,
                  input wire [31:0] b,
                  input wire cin,
                  output wire [31:0] sum,
                  output wire carry);

    assign {carry, sum} = a + b + cin;
endmodule





