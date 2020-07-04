// Top module of your design, you cannot modify this module!!
module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;

//=========================================
	// Note that the overall design of your MIPS includes:
	// 1. pipelined MIPS processor
	// 2. data cache
	// 3. instruction cache


	MIPS_Pipeline i_MIPS(
		// control interface
		.clk            (clk)           , 
		.rst_n          (rst_n)         ,
//----------I cache interface-------		
		.ICACHE_ren     (ICACHE_ren)    ,
		.ICACHE_wen     (ICACHE_wen)    ,
		.ICACHE_addr    (ICACHE_addr)   ,
		.ICACHE_wdata   (ICACHE_wdata)  ,
		.ICACHE_stall   (ICACHE_stall)  ,
		.ICACHE_rdata   (ICACHE_rdata)  ,
//----------D cache interface-------
		.DCACHE_ren     (DCACHE_ren)    ,
		.DCACHE_wen     (DCACHE_wen)    ,
		.DCACHE_addr    (DCACHE_addr)   ,
		.DCACHE_wdata   (DCACHE_wdata)  ,
		.DCACHE_stall   (DCACHE_stall)  ,
		.DCACHE_rdata   (DCACHE_rdata)
	);
	
	cache D_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (DCACHE_ren)  ,
        .proc_write (DCACHE_wen)  ,
        .proc_addr  (DCACHE_addr) ,
        .proc_rdata (DCACHE_rdata),
        .proc_wdata (DCACHE_wdata),
        .proc_stall (DCACHE_stall),
        .mem_read   (mem_read_D)  ,
        .mem_write  (mem_write_D) ,
        .mem_addr   (mem_addr_D)  ,
        .mem_wdata  (mem_wdata_D) ,
        .mem_rdata  (mem_rdata_D) ,
        .mem_ready  (mem_ready_D)
	);

	cache_I I_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (ICACHE_ren)  ,
        .proc_write (ICACHE_wen)  ,
        .proc_addr  (ICACHE_addr) ,
        .proc_rdata (ICACHE_rdata),
        .proc_wdata (ICACHE_wdata),
        .proc_stall (ICACHE_stall),
        .mem_read   (mem_read_I)  ,
        .mem_write  (mem_write_I) ,
        .mem_addr   (mem_addr_I)  ,
        .mem_wdata  (mem_wdata_I) ,
        .mem_rdata  (mem_rdata_I) ,
        .mem_ready  (mem_ready_I)
	);
endmodule


module MIPS_Pipeline(
    clk, 
		rst_n,
//----------I cache interface-------		
		ICACHE_ren, 
		ICACHE_wen,
		ICACHE_addr, 
		ICACHE_wdata, 
		ICACHE_stall,
		ICACHE_rdata,
//----------D cache interface-------
		DCACHE_ren, 
		DCACHE_wen, 
		DCACHE_addr, 
		DCACHE_wdata,
		DCACHE_stall,
		DCACHE_rdata);
    
input         clk, rst_n;
output        ICACHE_ren;
output        ICACHE_wen;
output [29:0] ICACHE_addr;
output [31:0] ICACHE_wdata;
input         ICACHE_stall;
input  [31:0] ICACHE_rdata;

output        DCACHE_ren;
output        DCACHE_wen;
output [29:0] DCACHE_addr;
output [31:0] DCACHE_wdata;
input         DCACHE_stall;
input  [31:0] DCACHE_rdata;

// wire declaration
wire          cache_stall, PC_write, J, RegWrite_MEMWB, Jal_IDEX, ALUSrc_IDEX, MemWrite_IDEX, MemRead_IDEX, MemtoReg_IDEX, RegWrite_IDEX, Branch, RegDst, MemWrite_EXMEM, MemRead_EXMEM, MemtoReg_EXMEM, RegWrite_EXMEM, MemtoReg_MEMWB;
wire   [1:0]  ForwardA_ID, ForwardB_ID, ForwardA_EX, ForwardB_EX;
wire   [4:0]  write_reg_MEMWB, write_reg_IDEX, Rs_IDEX, Rt_IDEX, Rd_IDEX, Rs_IFID, Rt_IFID, write_reg_EXMEM;
wire   [3:0]  ALU_ctrl_IDEX;
wire   [29:0] PC_IFID, PC_jump_w; 
wire   [31:0] instruction_IFID, write_data_MEMWB, write_data_EXMEM, write_data_IDEX, ALU_input1_IDEX, ALU_input2_IDEX, sign_extend_I_IDEX, store_data_EXMEM, write_data_MEMWB_old, memory_data_MEMWB;



// cell declaration


   assign cache_stall = ICACHE_stall | DCACHE_stall;
   
   IF if1(.clk(clk), .rst_n(rst_n), .ICACHE_ren(ICACHE_ren), .ICACHE_wen(ICACHE_wen), .ICACHE_addr(ICACHE_addr), .ICACHE_wdata(ICACHE_wdata), .cache_stall(cache_stall), .ICACHE_rdata(ICACHE_rdata), .instruction_IFID(instruction_IFID), .PC_IFID(PC_IFID), .PC_jump_w(PC_jump_w), .PC_write(PC_write), .J(J));
   
   ID id1( .clk(clk), .rst_n(rst_n), .cache_stall(cache_stall),
    .RegWrite_MEMWB(RegWrite_MEMWB),
    .instruction_IFID(instruction_IFID),
    .PC_IFID(PC_IFID),
    .write_reg_MEMWB(write_reg_MEMWB),
    .write_data_MEMWB(write_data_MEMWB),
    .write_data_EXMEM(write_data_EXMEM),
    .ALU_ctrl_IDEX(ALU_ctrl_IDEX), 
    .write_reg_IDEX(write_reg_IDEX), 
    .Rs_IDEX(Rs_IDEX), 
    .Rt_IDEX(Rt_IDEX), 
    .Rd_IDEX(Rd_IDEX),
    .write_data_IDEX(write_data_IDEX), 
    .ALU_input1_IDEX(ALU_input1_IDEX), 
    .ALU_input2_IDEX(ALU_input2_IDEX), 
    .sign_extend_I_IDEX(sign_extend_I_IDEX),
    .Jal_IDEX(Jal_IDEX), 
    .ALUSrc_IDEX(ALUSrc_IDEX), 
    .MemWrite_IDEX(MemWrite_IDEX), 
    .MemRead_IDEX(MemRead_IDEX), 
    .MemtoReg_IDEX(MemtoReg_IDEX), 
    .RegWrite_IDEX(RegWrite_IDEX),
    .flush_IDEX(PC_write),
    .PC_jump_w(PC_jump_w),
    .J(J),
    .ForwardA(ForwardA_ID),
    .ForwardB(ForwardB_ID),
    .Branch_w(Branch),
    .RegDst_w(RegDst),
    .Rs_IDEX_w(Rs_IFID),
    .Rt_IDEX_w(Rt_IFID));
   
   EX ex1(.clk(clk), .rst_n(rst_n), .cache_stall(cache_stall),
    .ALU_ctrl_IDEX(ALU_ctrl_IDEX),
    .write_reg_IDEX(write_reg_IDEX),
    .write_data_IDEX(write_data_IDEX), 
    .ALU_input1_IDEX(ALU_input1_IDEX), 
    .ALU_input2_IDEX(ALU_input2_IDEX), 
    .sign_extend_I_IDEX(sign_extend_I_IDEX),
    .Jal_IDEX(Jal_IDEX), 
    .ALUSrc_IDEX(ALUSrc_IDEX), 
    .MemWrite_IDEX(MemWrite_IDEX), 
    .MemRead_IDEX(MemRead_IDEX), 
    .MemtoReg_IDEX(MemtoReg_IDEX), 
    .RegWrite_IDEX(RegWrite_IDEX),
    .ForwardA(ForwardA_EX), 
    .ForwardB(ForwardB_EX),
    .write_data_MEMWB(write_data_MEMWB),
    .MemWrite_EXMEM(MemWrite_EXMEM), 
    .MemRead_EXMEM(MemRead_EXMEM), 
    .MemtoReg_EXMEM(MemtoReg_EXMEM), 
    .RegWrite_EXMEM(RegWrite_EXMEM),
    .write_reg_EXMEM(write_reg_EXMEM),
    .write_data_EXMEM(write_data_EXMEM), 
    .store_data_EXMEM(store_data_EXMEM));
   
   MEM mem1(.clk(clk), .rst_n(rst_n), .cache_stall(cache_stall),
    .MemWrite_EXMEM(MemWrite_EXMEM), 
    .MemRead_EXMEM(MemRead_EXMEM), 
    .MemtoReg_EXMEM(MemtoReg_EXMEM), 
    .RegWrite_EXMEM(RegWrite_EXMEM),
    .write_reg_EXMEM(write_reg_EXMEM),
    .write_data_EXMEM(write_data_EXMEM), 
    .store_data_EXMEM(store_data_EXMEM),
		.DCACHE_ren(DCACHE_ren), 
		.DCACHE_wen(DCACHE_wen),
		.DCACHE_addr(DCACHE_addr), 
		.DCACHE_wdata(DCACHE_wdata), 
		.DCACHE_rdata(DCACHE_rdata),
    .MemtoReg_MEMWB(MemtoReg_MEMWB), 
    .RegWrite_MEMWB(RegWrite_MEMWB),
    .memory_data_MEMWB(memory_data_MEMWB),
    .write_reg_MEMWB(write_reg_MEMWB),
    .write_data_MEMWB_old(write_data_MEMWB_old));
   
   WB wb1(    
    .MemtoReg_MEMWB(MemtoReg_MEMWB),
    .memory_data_MEMWB(memory_data_MEMWB),
    .write_data_MEMWB_old(write_data_MEMWB_old),
    .write_data_MEMWB(write_data_MEMWB));
    
    Hazard_unit hazard1(.MemRead_IDEX(MemRead_IDEX), .write_reg_IDEX(write_reg_IDEX), .write_reg_EXMEM(write_reg_EXMEM), .Rs_IFID(Rs_IFID), .Rt_IFID(Rt_IFID), .PC_write(PC_write), .Branch(Branch), .RegWrite_IDEX(RegWrite_IDEX), .MemRead_EXMEM(MemRead_EXMEM), .stall(cache_stall));
    
    Forwarding_unit2 fID(.Rs_IDEX(Rs_IFID), .Rt_IDEX(Rt_IFID), .Rd_EXMEM(write_reg_EXMEM), .Rd_MEMWB(write_reg_MEMWB), .RegWrite_EXMEM(RegWrite_EXMEM), .RegWrite_MEMWB(RegWrite_MEMWB), .ForwardA(ForwardA_ID), .ForwardB(ForwardB_ID));
    Forwarding_unit fEX(.Rs_IDEX(Rs_IDEX), .Rt_IDEX(Rt_IDEX), .Rd_EXMEM(write_reg_EXMEM), .Rd_MEMWB(write_reg_MEMWB), .RegWrite_EXMEM(RegWrite_EXMEM), .RegWrite_MEMWB(RegWrite_MEMWB), .ForwardA(ForwardA_EX), .ForwardB(ForwardB_EX));
    
endmodule


module IF(    
    clk, 
		rst_n,
		ICACHE_ren, 
		ICACHE_wen,
		ICACHE_addr, 
		ICACHE_wdata, 
		cache_stall,
		ICACHE_rdata,
    instruction_IFID,
    PC_IFID,
    PC_jump_w,
    PC_write,
    J);
input         clk, rst_n;
output        ICACHE_ren;
output        ICACHE_wen;
output [29:0] ICACHE_addr;
output [31:0] ICACHE_wdata;
input         cache_stall, J, PC_write;
input  [31:0] ICACHE_rdata;
input  [29:0] PC_jump_w;

output reg [31:0] instruction_IFID;
output reg [29:0] PC_IFID;
wire       [31:0] instruction_IFID_w;
wire       [29:0] PC_IFID_w; 
reg               ren_r; 


    assign ICACHE_ren = ren_r;
    assign ICACHE_wen = 1'b0;
    assign ICACHE_addr = PC_IFID;
    assign PC_IFID_w = (PC_write || cache_stall) ? PC_IFID :
                       (J) ? PC_jump_w : (PC_IFID + 30'b1);

    assign instruction_IFID_w = (PC_write || cache_stall) ? instruction_IFID : (J) ? 32'b0 : ICACHE_rdata;
    
    always@(posedge clk) begin
        if (! rst_n) begin
            PC_IFID <= 30'b0;
            instruction_IFID <= 32'b0;
            ren_r <= 1'b0;
        end
        else begin
            PC_IFID <= PC_IFID_w;
            instruction_IFID <= instruction_IFID_w;
            ren_r <= 1'b1;
        end
    end
endmodule


module ID(    
    clk, 
    rst_n,
    cache_stall,
    RegWrite_MEMWB,
    instruction_IFID,
    PC_IFID,
    write_reg_MEMWB,
    write_data_MEMWB,
    write_data_EXMEM,
    ALU_ctrl_IDEX, 
    write_reg_IDEX, 
    Rs_IDEX, 
    Rt_IDEX, 
    Rd_IDEX,
    write_data_IDEX, 
    ALU_input1_IDEX, 
    ALU_input2_IDEX, 
    sign_extend_I_IDEX,
    Jal_IDEX, 
    ALUSrc_IDEX, 
    MemWrite_IDEX, 
    MemRead_IDEX, 
    MemtoReg_IDEX, 
    RegWrite_IDEX,
    flush_IDEX,
    PC_jump_w,
    J,
    ForwardA,
    ForwardB,
    Branch_w,
    RegDst_w,
    Rs_IDEX_w,
    Rt_IDEX_w);
input         clk, rst_n, cache_stall, RegWrite_MEMWB, flush_IDEX;
input  [1:0]  ForwardA, ForwardB;
input  [31:0] instruction_IFID;
input  [29:0] PC_IFID;
input  [4:0]  write_reg_MEMWB;
input  [31:0] write_data_MEMWB, write_data_EXMEM;
output reg [4:0]  write_reg_IDEX, Rs_IDEX, Rt_IDEX, Rd_IDEX, Rs_IDEX_w, Rt_IDEX_w;
output reg [3:0]  ALU_ctrl_IDEX;
output reg [31:0] write_data_IDEX, ALU_input1_IDEX, ALU_input2_IDEX, sign_extend_I_IDEX;
output reg Jal_IDEX, ALUSrc_IDEX, MemWrite_IDEX, MemRead_IDEX, MemtoReg_IDEX, RegWrite_IDEX;
output [29:0] PC_jump_w;
output J, Branch_w, RegDst_w;

    wire RegDst_w, Jump_w, Branch_w;  // no need to go forward
    wire Jal_w, ALUSrc_w;  // go forward EX stage
    wire MemWrite_w, MemRead_w;  // go forward MEM stage
    wire MemtoReg_w, RegWrite_w; // go forward WB stage
    wire [3:0] ALU_ctrl_w; // go forward EX stage
    wire [31:0] read_data1, read_data2, sign_extend_I_w, branch_in1, branch_in2;
    
    reg [4:0]  Rd_IDEX_w, write_reg_IDEX_w;
    reg [3:0]  ALU_ctrl_IDEX_w;
    reg [31:0] ALU_input1_IDEX_w, ALU_input2_IDEX_w, sign_extend_I_IDEX_w, write_data_IDEX_w;
    reg        Jal_IDEX_w, ALUSrc_IDEX_w, MemWrite_IDEX_w, MemRead_IDEX_w, MemtoReg_IDEX_w, RegWrite_IDEX_w;


    Ctrl control_unit(.opcode(instruction_IFID[31:26]), .RegDst(RegDst_w), .Jump(Jump_w), .Branch(Branch_w), .MemWrite(MemWrite_w), .MemRead(MemRead_w), .MemtoReg(MemtoReg_w), .ALUSrc(ALUSrc_w), .RegWrite(RegWrite_w), .Jal(Jal_w), .funct(instruction_IFID[5:0]), .ALU_ctrl(ALU_ctrl_w));
    
    Registers rgs(.read_data1(read_data1), .read_data2(read_data2), .read_reg1(instruction_IFID[25:21]), .read_reg2(instruction_IFID[20:16]), .write_reg(write_reg_MEMWB), .write_data(write_data_MEMWB), .clk(clk), .rst_n(rst_n), .RegWrite_r(RegWrite_MEMWB));
    
    assign sign_extend_I_w = { {16{instruction_IFID[15]}}, {instruction_IFID[15:0]} };


    assign branch_in1 = (ForwardA[1] == 1'b1) ? write_data_EXMEM : read_data1;
    assign branch_in2 = (ForwardB[1] == 1'b1) ? write_data_EXMEM : read_data2;
    PCs pc1(.Branch(Branch_w), .beqorbne(~ALU_ctrl_w[0]), .read_data1(branch_in1), .read_data2(branch_in2), .sign_extend_I(sign_extend_I_w), .Jump(Jump_w), .Instruction(instruction_IFID[25:0]), .RegDst(RegDst_w), .PC_w(PC_IFID), .PC_jump_w(PC_jump_w), .J(J));

    always@(*) begin
        if (cache_stall) begin
            ALU_ctrl_IDEX_w = ALU_ctrl_IDEX;
            Rs_IDEX_w = Rs_IDEX;
            Rt_IDEX_w = Rt_IDEX;
            Rd_IDEX_w = Rd_IDEX;
            ALU_input1_IDEX_w = ALU_input1_IDEX;
            ALU_input2_IDEX_w = ALU_input2_IDEX;
            sign_extend_I_IDEX_w = sign_extend_I_IDEX;
            Jal_IDEX_w = Jal_IDEX;
            ALUSrc_IDEX_w = ALUSrc_IDEX;
            MemWrite_IDEX_w = MemWrite_IDEX;
            MemRead_IDEX_w = MemRead_IDEX;
            MemtoReg_IDEX_w = MemtoReg_IDEX;
            RegWrite_IDEX_w = RegWrite_IDEX;
            write_reg_IDEX_w = write_reg_IDEX;
            write_data_IDEX_w = write_data_IDEX;
        end
        else begin
            ALU_ctrl_IDEX_w = ALU_ctrl_w;
            Rs_IDEX_w = instruction_IFID[25:21];
            Rt_IDEX_w = instruction_IFID[20:16];
            Rd_IDEX_w = instruction_IFID[15:11];
            ALU_input1_IDEX_w = read_data1;
            ALU_input2_IDEX_w = read_data2;
            sign_extend_I_IDEX_w = sign_extend_I_w;
            Jal_IDEX_w = Jal_w;
            ALUSrc_IDEX_w = ALUSrc_w;
            MemWrite_IDEX_w = MemWrite_w;
            MemRead_IDEX_w = MemRead_w;
            MemtoReg_IDEX_w = MemtoReg_w;
            RegWrite_IDEX_w = RegWrite_w;
            write_reg_IDEX_w = (Jal_w && instruction_IFID[27])  ? 5'b11111 :
                               (RegDst_w) ? instruction_IFID[15:11] : instruction_IFID[20:16];
            write_data_IDEX_w = {PC_IFID, 2'b00};            
        end
    end

    always@(posedge clk) begin
        if ((! rst_n) || (flush_IDEX && (~cache_stall))) begin
            ALU_ctrl_IDEX <= 4'b0;
            Rs_IDEX <= 5'b0;
            Rt_IDEX <= 5'b0;
            Rd_IDEX <= 5'b0;
            ALU_input1_IDEX <= 32'b0; 
            ALU_input2_IDEX <= 32'b0;
            sign_extend_I_IDEX <= 32'b0;
            Jal_IDEX <= 1'b0;
            ALUSrc_IDEX <= 1'b0;
            MemWrite_IDEX <= 1'b0;
            MemRead_IDEX <= 1'b0;
            MemtoReg_IDEX <= 1'b0;
            RegWrite_IDEX <= 1'b0;
            write_reg_IDEX <= 5'b0;
            write_data_IDEX <= 32'b0; 
        end
        else begin
            ALU_ctrl_IDEX <= ALU_ctrl_IDEX_w;
            Rs_IDEX <= Rs_IDEX_w;
            Rt_IDEX <= Rt_IDEX_w;
            Rd_IDEX <= Rd_IDEX_w;
            ALU_input1_IDEX <= ALU_input1_IDEX_w;
            ALU_input2_IDEX <= ALU_input2_IDEX_w;
            sign_extend_I_IDEX <= sign_extend_I_IDEX_w;
            Jal_IDEX <= Jal_IDEX_w;
            ALUSrc_IDEX <= ALUSrc_IDEX_w;
            MemWrite_IDEX <= MemWrite_IDEX_w;
            MemRead_IDEX <= MemRead_IDEX_w;
            MemtoReg_IDEX <= MemtoReg_IDEX_w;
            RegWrite_IDEX <= RegWrite_IDEX_w;
            write_reg_IDEX <= write_reg_IDEX_w;
            write_data_IDEX <= write_data_IDEX_w;
        end
    end
endmodule


module PCs(Branch, beqorbne, read_data1, read_data2, sign_extend_I, Jump, Instruction, RegDst, PC_w, PC_jump_w, J);
    input             Branch, beqorbne, Jump, RegDst;
    input      [29:0] PC_w;
    input      [31:0] read_data1, read_data2, sign_extend_I;
    input      [25:0] Instruction;
    output reg [29:0] PC_jump_w;
    output reg        J;
    wire Zero_out, branching;
    assign Zero_out = (read_data1 == read_data2) ? 1'b1 : 1'b0;
    assign branching = (beqorbne) ? Zero_out : ~Zero_out;
    always@(*) begin
        J = 1'b0;
        PC_jump_w = PC_w;
        if (Branch && (branching)) begin
            PC_jump_w = PC_w + sign_extend_I;
            J = 1'b1;
        end
        else if (Jump) begin
            PC_jump_w = {PC_w[29:26], Instruction[25:0]};
            J = 1'b1;
        end
        else if (RegDst && (Instruction[3:1] == 3'b100)) begin  // jr or jalr
            PC_jump_w = read_data1[31:2];
            J = 1'b1;
        end
        else
            PC_jump_w = PC_w + 30'b1;
    end


endmodule

module Hazard_unit(MemRead_IDEX, write_reg_IDEX, write_reg_EXMEM, Rs_IFID, Rt_IFID, PC_write, Branch, RegWrite_IDEX, MemRead_EXMEM, stall);
input MemRead_IDEX, Branch, RegWrite_IDEX, MemRead_EXMEM, stall;
input [4:0] write_reg_IDEX, write_reg_EXMEM, Rs_IFID, Rt_IFID;
output reg PC_write;
    always@(*) begin
        if (stall)
            PC_write = 1'b0;
        else if ((MemRead_IDEX || (RegWrite_IDEX && Branch)) && ((Rs_IFID == write_reg_IDEX) || (Rt_IFID == write_reg_IDEX)))
            PC_write = 1'b1;
        else if (MemRead_EXMEM && Branch && ((Rs_IFID == write_reg_EXMEM) || (Rt_IFID == write_reg_EXMEM)))
            PC_write = 1'b1;
        else
            PC_write = 1'b0;
    end
endmodule

module Ctrl(opcode, RegDst, Jump, Branch, MemWrite, MemRead, MemtoReg, ALUSrc, RegWrite, Jal, funct, ALU_ctrl);
    input [5:0] opcode;
    input [5:0] funct;
    output reg RegDst, Jump, Branch, MemWrite, MemRead, MemtoReg, ALUSrc, RegWrite, Jal;
    output reg [3:0] ALU_ctrl;
  

    always@(*) begin
        RegDst = 1'b0;  
        Jump = 1'b0;
        Branch = 1'b0;
        MemWrite = 1'b0;
        MemRead = 1'b0;
        MemtoReg = 1'b0;
        ALUSrc = 1'b0;
        RegWrite = 1'b0;
        Jal = 1'b0;
        ALU_ctrl = 4'b0000;
        case (opcode)
            6'h00: begin  // R format
                RegDst = 1'b1;  
                // Jump = 1'b0;
                // Branch = 1'b0;
                // MemWrite = 1'b0;
                // MemRead = 1'b0;
                // MemtoReg = 1'b0;
                // ALUSrc = 1'b0;
                RegWrite = 1'b1;
                // Jal = 1'b0;
                case({funct[5], {funct[3:0]}})
                    5'h10:   ALU_ctrl = 4'b0001;  // add
                    5'h12:   ALU_ctrl = 4'b0010;  // sub
                    5'h14:   ALU_ctrl = 4'b0011;  // and
                    5'h08: begin                  // jr
                       ALU_ctrl = 4'b0100;
                       RegWrite = 1'b0;
                    end
                    5'h15:   ALU_ctrl = 4'b0101;  // or
                    5'h17:   ALU_ctrl = 4'b1101;  // nor
                    5'h1a:   ALU_ctrl = 4'b0110;  // slt
                    5'h16:   ALU_ctrl = 4'b0111;  // xor
                    5'h00:   ALU_ctrl = 4'b1001;  // sll
                    5'h02:   ALU_ctrl = 4'b1000;  // srl
                    5'h03:   ALU_ctrl = 4'b1011;  // sra
                    5'h09: begin
                        ALU_ctrl = 4'b0100;  // jalr
                        Jal = 1'b1;
                    end
                    default: ALU_ctrl = 4'b0000;
                endcase
            end
            6'h08: begin  // addi
                // RegDst = 1'b0;
                // Jump = 1'b0;
                // Branch = 1'b0;
                // MemWrite = 1'b0;
                // MemRead = 1'b0;
                // MemtoReg = 1'b0;
                ALUSrc = 1'b1;
                RegWrite = 1'b1;
                // Jal = 1'b0;
                ALU_ctrl = 4'b0001;
            end
            6'h0c: begin  // andi
                // RegDst = 1'b0;
                // Jump = 1'b0;
                // Branch = 1'b0;
                // MemWrite = 1'b0;
                // MemRead = 1'b0;
                // MemtoReg = 1'b0;
                ALUSrc = 1'b1;
                RegWrite = 1'b1;
                // Jal = 1'b0;
                ALU_ctrl = 4'b0011;
            end
            6'h04: begin  // beq
                // RegDst = 1'b0;  
                // Jump = 1'b0;
                Branch = 1'b1;
                // MemWrite = 1'b0;
                // MemRead = 1'b0;
                // MemtoReg = 1'b0;
                // ALUSrc = 1'b0;
                // RegWrite = 1'b0;
                // Jal = 1'b0;
                ALU_ctrl = 4'b0010;
            end
            6'h05: begin  // bne
                // RegDst = 1'b0;  
                // Jump = 1'b0;
                Branch = 1'b1;
                // MemWrite = 1'b0;
                // MemRead = 1'b0;
                // MemtoReg = 1'b0;
                // ALUSrc = 1'b0;
                // RegWrite = 1'b0;
                // Jal = 1'b0;
                ALU_ctrl = 4'b1111;
            end
            6'h02: begin // Jump
                // RegDst = 1'b0;
                Jump = 1'b1;
                // Branch = 1'b0;
                // MemWrite = 1'b0;
                // MemRead = 1'b0;
                // MemtoReg = 1'b0;
                // ALUSrc = 1'b0;
                // RegWrite = 1'b0;
                // Jal = 1'b0;
                // ALU_ctrl = 4'b0000;
            end
            6'h03: begin  // Jal
                // RegDst = 1'b0;  
                Jump = 1'b1;
                // Branch = 1'b0;
                // MemWrite = 1'b0;
                // MemRead = 1'b0;
                // MemtoReg = 1'b0;
                // ALUSrc = 1'b0;
                RegWrite = 1'b1;
                Jal = 1'b1;
                // ALU_ctrl = 4'b0000;
            end
            6'h23: begin  // lw
                // RegDst = 1'b0;
                // Jump = 1'b0;
                // Branch = 1'b0;
                // MemWrite = 1'b0;
                MemRead = 1'b1;
                MemtoReg = 1'b1;
                ALUSrc = 1'b1;
                RegWrite = 1'b1;
                // Jal = 1'b0;
                ALU_ctrl = 4'b0001;
            end
            6'h0d: begin  // ori
                // RegDst = 1'b0;
                // Jump = 1'b0;
                // Branch = 1'b0;
                // MemWrite = 1'b0;
                // MemRead = 1'b0;
                // MemtoReg = 1'b0;
                ALUSrc = 1'b1;
                RegWrite = 1'b1;
                // Jal = 1'b0;
                ALU_ctrl = 4'b0101;
            end
            6'h0a: begin  // slti
                // RegDst = 1'b0;
                // Jump = 1'b0;
                // Branch = 1'b0;
                // MemWrite = 1'b0;
                // MemRead = 1'b0;
                // MemtoReg = 1'b0;
                ALUSrc = 1'b1;
                RegWrite = 1'b1;
                // Jal = 1'b0;
                ALU_ctrl = 4'b0110;
            end
            6'h0e: begin  // xori
                // RegDst = 1'b0;
                // Jump = 1'b0;
                // Branch = 1'b0;
                // MemWrite = 1'b0;
                // MemRead = 1'b0;
                // MemtoReg = 1'b0;
                ALUSrc = 1'b1;
                RegWrite = 1'b1;
                // Jal = 1'b0;
                ALU_ctrl = 4'b0111;
            end
            6'h2b: begin  // sw
                // RegDst = 1'b0;
                // Jump = 1'b0;
                // Branch = 1'b0;
                MemWrite = 1'b1;
                // MemRead = 1'b0;
                // MemtoReg = 1'b0;
                ALUSrc = 1'b1;
                // RegWrite = 1'b0;
                // Jal = 1'b0;
                ALU_ctrl = 4'b0001;
            end
            default: begin
                RegDst = 1'b0;
                Jump = 1'b0;
                Branch = 1'b0;
                MemWrite = 1'b0;
                MemRead = 1'b0;
                MemtoReg = 1'b0;
                ALUSrc = 1'b0;
                RegWrite = 1'b0;
                Jal = 1'b0;
                ALU_ctrl = 4'b0000;
            end
        endcase
    end  

endmodule


module Registers(read_data1, read_data2, read_reg1, read_reg2, write_reg, write_data, clk, rst_n, RegWrite_r);
    input [4:0] read_reg1, read_reg2, write_reg;
    input [31:0] write_data;
    input clk, rst_n, RegWrite_r;
    output [31:0] read_data1, read_data2;

    reg [31:0] Regs_r [31:0];
    reg [31:0] Regs_w [31:0];
    integer i;
    
    assign read_data1 = Regs_w[read_reg1];
    assign read_data2 = Regs_w[read_reg2];
    
    always@(*) begin
        for (i=0; i<=31; i=i+1)
            Regs_w[i] = Regs_r[i];
        if (RegWrite_r)
            Regs_w[write_reg] = write_data;
    end
    
    always@(posedge clk) begin
        if (!rst_n) begin
            for (i=0; i<=31; i=i+1)
                Regs_r[i] <= 32'b0;
        end
        else begin
            Regs_r[0] <= 32'b0;
            for (i=1; i<=31; i=i+1)
                Regs_r[i] <= Regs_w[i];
        end
    end
    
    
endmodule

module EX(    
    clk, 
    rst_n,
    cache_stall,
    ALU_ctrl_IDEX,
    write_reg_IDEX,
    write_data_IDEX, 
    ALU_input1_IDEX, 
    ALU_input2_IDEX, 
    sign_extend_I_IDEX,
    Jal_IDEX, 
    ALUSrc_IDEX, 
    MemWrite_IDEX, 
    MemRead_IDEX, 
    MemtoReg_IDEX, 
    RegWrite_IDEX,
    ForwardA, 
    ForwardB, 
    write_data_MEMWB,
    MemWrite_EXMEM, 
    MemRead_EXMEM, 
    MemtoReg_EXMEM, 
    RegWrite_EXMEM,
    write_reg_EXMEM,
    write_data_EXMEM, 
    store_data_EXMEM);
input         clk, rst_n, cache_stall;
input [4:0]   write_reg_IDEX;
input [3:0]   ALU_ctrl_IDEX;
input [31:0]  write_data_IDEX, ALU_input1_IDEX, ALU_input2_IDEX, sign_extend_I_IDEX;
input         Jal_IDEX, ALUSrc_IDEX;
input         MemWrite_IDEX, MemRead_IDEX, MemtoReg_IDEX, RegWrite_IDEX;
input  [1:0]  ForwardA, ForwardB;
input  [31:0] write_data_MEMWB;
output reg        MemWrite_EXMEM, MemRead_EXMEM, MemtoReg_EXMEM, RegWrite_EXMEM;
output reg [4:0]  write_reg_EXMEM;
output reg [31:0] write_data_EXMEM, store_data_EXMEM;
    
    wire [31:0] ALU_input1, ALU_input2, ALU_output;
    reg  [31:0] Forward_in1, Forward_in2;
    reg         MemWrite_EXMEM_w, MemRead_EXMEM_w, MemtoReg_EXMEM_w, RegWrite_EXMEM_w;
    reg  [4:0]  write_reg_EXMEM_w;
    reg  [31:0] write_data_EXMEM_w, store_data_EXMEM_w;
    
    
    assign ALU_input1 = (ALU_ctrl_IDEX[3:2] == 2'b10) ? {27'b0, sign_extend_I_IDEX[10:6]} : Forward_in1;
    assign ALU_input2 = (ALUSrc_IDEX) ? sign_extend_I_IDEX : Forward_in2;     
    
    ALU alu(.ALU_in1(ALU_input1), .ALU_in2(ALU_input2), .ALU_ctrl(ALU_ctrl_IDEX), .ALU_output(ALU_output));

    
    always@(*) begin
        if (cache_stall) begin
            MemWrite_EXMEM_w = MemWrite_EXMEM;
            MemRead_EXMEM_w = MemRead_EXMEM;
            MemtoReg_EXMEM_w = MemtoReg_EXMEM;
            RegWrite_EXMEM_w = RegWrite_EXMEM;
            write_reg_EXMEM_w = write_reg_EXMEM;
            write_data_EXMEM_w = write_data_EXMEM;
            store_data_EXMEM_w = store_data_EXMEM;
        end
        else begin
            MemWrite_EXMEM_w = MemWrite_IDEX;
            MemRead_EXMEM_w = MemRead_IDEX;
            MemtoReg_EXMEM_w = MemtoReg_IDEX;
            RegWrite_EXMEM_w = RegWrite_IDEX;
            write_reg_EXMEM_w = write_reg_IDEX;
            write_data_EXMEM_w = (Jal_IDEX) ? write_data_IDEX : ALU_output;
            store_data_EXMEM_w = (ForwardB == 2'b10) ? write_data_EXMEM :
                                 (ForwardB == 2'b01) ? write_data_MEMWB : ALU_input2_IDEX;
        end
        case (ForwardA)
            2'b10:   Forward_in1 = write_data_EXMEM;
            2'b01:   Forward_in1 = write_data_MEMWB;
            default: Forward_in1 = ALU_input1_IDEX;
        endcase
        case (ForwardB)
            2'b10:   Forward_in2 = write_data_EXMEM;
            2'b01:   Forward_in2 = write_data_MEMWB;
            default: Forward_in2 = ALU_input2_IDEX;
        endcase
    end
    always@(posedge clk) begin
        if (! rst_n) begin
            MemWrite_EXMEM <= 1'b0;
            MemRead_EXMEM <= 1'b0;
            MemtoReg_EXMEM <= 1'b0;
            RegWrite_EXMEM <= 1'b0;
            write_reg_EXMEM <= 5'b0;
            write_data_EXMEM <= 32'b0;
            store_data_EXMEM <= 32'b0;
        end
        else begin
            MemWrite_EXMEM <= MemWrite_EXMEM_w;
            MemRead_EXMEM <= MemRead_EXMEM_w;
            MemtoReg_EXMEM <= MemtoReg_EXMEM_w;
            RegWrite_EXMEM <= RegWrite_EXMEM_w;
            write_reg_EXMEM <= write_reg_EXMEM_w;
            write_data_EXMEM <= write_data_EXMEM_w;
            store_data_EXMEM <= store_data_EXMEM_w;
        end
    end
endmodule


module ALU(ALU_in1, ALU_in2, ALU_ctrl, ALU_output);
    input  [31:0] ALU_in1, ALU_in2;
    input  [3:0] ALU_ctrl;
    output reg [31:0] ALU_output;
    always@(*) begin
        ALU_output = 32'b0;
        case(ALU_ctrl)
            4'b0001:  ALU_output = $signed(ALU_in1) + $signed(ALU_in2); // add
            4'b0010:  ALU_output = $signed(ALU_in1) - $signed(ALU_in2);
            4'b0011:  ALU_output = ALU_in1 & ALU_in2;
            4'b0100:  ALU_output = ALU_in1;
            4'b0101:  ALU_output = ALU_in1 | ALU_in2;
            4'b1101:  ALU_output = ~(ALU_in1 | ALU_in2);
            4'b0110:  ALU_output[0] = ($signed(ALU_in1) < $signed(ALU_in2)) ? 1'b1 : 1'b0;
            4'b0111:  ALU_output = ALU_in1 ^ ALU_in2;
            4'b1000:  ALU_output = ALU_in2 >> ALU_in1[4:0];
            4'b1011:  ALU_output = $signed(ALU_in2) >>> ALU_in1[4:0];
            4'b1001:  ALU_output = ALU_in2 << ALU_in1[4:0];
            default:  ALU_output = 32'b0;
        endcase
    end

endmodule


module Forwarding_unit(Rs_IDEX, Rt_IDEX, Rd_EXMEM, Rd_MEMWB, RegWrite_EXMEM, RegWrite_MEMWB, ForwardA, ForwardB);
input   [4:0]  Rs_IDEX, Rt_IDEX, Rd_EXMEM, Rd_MEMWB;
input          RegWrite_EXMEM, RegWrite_MEMWB;
output  [1:0]  ForwardA, ForwardB;
    
    assign ForwardA = (RegWrite_EXMEM && Rd_EXMEM && (Rd_EXMEM == Rs_IDEX)) ? 2'b10 :
               (RegWrite_MEMWB && Rd_MEMWB && (Rd_MEMWB == Rs_IDEX)) ? 2'b01 : 2'b00;
    assign ForwardB = (RegWrite_EXMEM && Rd_EXMEM && (Rd_EXMEM == Rt_IDEX)) ? 2'b10 :
               (RegWrite_MEMWB && Rd_MEMWB && (Rd_MEMWB == Rt_IDEX)) ? 2'b01 : 2'b00;
endmodule

module Forwarding_unit2(Rs_IDEX, Rt_IDEX, Rd_EXMEM, Rd_MEMWB, RegWrite_EXMEM, RegWrite_MEMWB, ForwardA, ForwardB);
input   [4:0]  Rs_IDEX, Rt_IDEX, Rd_EXMEM, Rd_MEMWB;
input          RegWrite_EXMEM, RegWrite_MEMWB;
output  [1:0]  ForwardA, ForwardB;
    
    assign ForwardA = (RegWrite_EXMEM && Rd_EXMEM && (Rd_EXMEM == Rs_IDEX)) ? 2'b10 : 2'b00;
    assign ForwardB = (RegWrite_EXMEM && Rd_EXMEM && (Rd_EXMEM == Rt_IDEX)) ? 2'b10 : 2'b00;
endmodule


module MEM(    
    clk, 
		rst_n,
    cache_stall,
    MemWrite_EXMEM, 
    MemRead_EXMEM, 
    MemtoReg_EXMEM, 
    RegWrite_EXMEM,
    write_reg_EXMEM,
    write_data_EXMEM, 
    store_data_EXMEM,
		DCACHE_ren, 
		DCACHE_wen,
		DCACHE_addr, 
		DCACHE_wdata, 
		DCACHE_rdata,
    MemtoReg_MEMWB, 
    RegWrite_MEMWB,
    memory_data_MEMWB,
    write_reg_MEMWB,
    write_data_MEMWB_old);
input         clk, rst_n, cache_stall;
output        DCACHE_ren;
output        DCACHE_wen;
output [29:0] DCACHE_addr;
output [31:0] DCACHE_wdata;
input  [31:0] DCACHE_rdata;

input         MemWrite_EXMEM, MemRead_EXMEM, MemtoReg_EXMEM, RegWrite_EXMEM;
input  [4:0]  write_reg_EXMEM;
input  [31:0] write_data_EXMEM, store_data_EXMEM;

output reg        MemtoReg_MEMWB, RegWrite_MEMWB;
output reg [31:0] memory_data_MEMWB;
output reg [4:0]  write_reg_MEMWB;
output reg [31:0] write_data_MEMWB_old;

    reg        MemtoReg_MEMWB_w, RegWrite_MEMWB_w;
    reg [31:0] memory_data_MEMWB_w;
    reg [4:0]  write_reg_MEMWB_w;
    reg [31:0] write_data_MEMWB_old_w;

    assign DCACHE_addr = write_data_EXMEM[31:2];
    assign DCACHE_ren = MemRead_EXMEM;
    assign DCACHE_wen = MemWrite_EXMEM;
    assign DCACHE_wdata = store_data_EXMEM;

    always@(*) begin
        if (cache_stall) begin
            MemtoReg_MEMWB_w = MemtoReg_MEMWB;
            RegWrite_MEMWB_w = RegWrite_MEMWB;
            memory_data_MEMWB_w = memory_data_MEMWB;
            write_reg_MEMWB_w = write_reg_MEMWB;
            write_data_MEMWB_old_w = write_data_MEMWB_old;
        end
        else begin
            MemtoReg_MEMWB_w = MemtoReg_EXMEM;
            RegWrite_MEMWB_w = RegWrite_EXMEM;
            memory_data_MEMWB_w = DCACHE_rdata;
            write_reg_MEMWB_w = write_reg_EXMEM;
            write_data_MEMWB_old_w = write_data_EXMEM;
        end
    end

    always@(posedge clk) begin
        if (! rst_n) begin
            MemtoReg_MEMWB <= 1'b0;
            RegWrite_MEMWB <= 1'b0;
            memory_data_MEMWB <= 32'b0;
            write_reg_MEMWB <= 5'b0;
            write_data_MEMWB_old <= 32'b0;
        end
        else begin
            MemtoReg_MEMWB <= MemtoReg_MEMWB_w;
            RegWrite_MEMWB <= RegWrite_MEMWB_w;
            memory_data_MEMWB <= memory_data_MEMWB_w;
            write_reg_MEMWB <= write_reg_MEMWB_w;
            write_data_MEMWB_old <= write_data_MEMWB_old_w;
        end
    end
endmodule



module WB(    
    MemtoReg_MEMWB,
    memory_data_MEMWB,
    write_data_MEMWB_old,
    write_data_MEMWB);
input         MemtoReg_MEMWB;
input  [31:0] write_data_MEMWB_old, memory_data_MEMWB;

output [31:0] write_data_MEMWB;

    assign write_data_MEMWB = (MemtoReg_MEMWB) ? memory_data_MEMWB : write_data_MEMWB_old;

endmodule




module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    
//==== wire/reg definition ================================

    parameter IDLE      = 2'b00;
    parameter COMPARE   = 2'b00;
    parameter W_BACK    = 2'b10;
    parameter ALLOCATE  = 2'b11;

    parameter VALIDBIT  = 1;
    parameter TAGWIDTH  = 25;
    parameter DIRTYBIT  = 1;
    parameter DATAWIDTH = 128;

    integer i, j;

    reg         proc_stall, mem_read, mem_write;
    reg [27 :0] mem_addr;
    reg [127:0] mem_wdata;
    reg [31 :0] proc_rdata;

    reg [VALIDBIT + TAGWIDTH + DIRTYBIT + DATAWIDTH - 1 : 0] block [0:7];
    reg [VALIDBIT + TAGWIDTH + DIRTYBIT + DATAWIDTH - 1 : 0] block_nxt [0:7];
    
    reg [1:0] state;
    reg [1:0] state_nxt;


    reg [127:0]  mem_rdata_r;
    reg          mem_ready_r;
    reg         valid, dirty, hit;
    wire [1:0]  block_offset;
    wire [2:0]  block_index;
    wire [24:0] proc_tag;

//==== finite state machine  =============================

    always@(*) begin
	state_nxt = IDLE;
	case(state)
	    COMPARE: begin
		if (proc_read || proc_write) begin
		    if(hit && valid)
		    	state_nxt = COMPARE;
		    else if (dirty) 
		    	state_nxt = W_BACK;
		    else 
		    	state_nxt = ALLOCATE;
		end
		else
		    state_nxt = COMPARE;
	    end

	    W_BACK: begin
		if (!mem_ready_r)
		    state_nxt = W_BACK;
		else
		    state_nxt = ALLOCATE;
	    end
	    ALLOCATE: begin
		if (!mem_ready_r)
		    state_nxt = ALLOCATE;
		else
		    state_nxt = COMPARE;
	    end
	endcase
    end

//==== combinational circuit ==============================

    assign block_offset = proc_addr[1:0];
    assign block_index = proc_addr[4:2];
    assign proc_tag = proc_addr[29:5];

    always@(*) begin/// control unit
	proc_stall = 0;
	valid = 0;
	hit = 0;
	mem_addr  = 28'h0000000;
	mem_read  = 0;
	mem_write = 0;
	mem_wdata = 0;
	dirty     = 1'b0;
	case(state)
	    COMPARE: begin
		valid = block[block_index][154];
		dirty = block[block_index][128];
		hit   = (proc_tag == block[block_index][153:129]);
		if((hit && valid) || (!(proc_read || proc_write)))
		    proc_stall = 0;
		else
		    proc_stall = 1;
	    end

	    W_BACK: begin
		mem_addr  = {block[block_index][153:129], block_index};
		mem_read  = 0;
		mem_write = 1;
		mem_wdata = block[block_index][127:0];
		proc_stall = 1;
	    end

	    ALLOCATE: begin
		proc_stall = 1'b1;
		mem_read  = (mem_ready_r) ? 1'b0 : 1'b1;
		mem_write = 0;
		mem_addr  = proc_addr[29:2];
	    end
	endcase
    end

    always@(*) begin/// data flow
	proc_rdata = 32'h0;
	for (i = 0; i < 8; i = i + 1) begin
	    block_nxt[i] = block[i];
	end
	case(state)
	    COMPARE: begin
		if(hit && valid && proc_read) begin
		    case(block_offset)
			2'b11: proc_rdata = block[block_index][127:96];
			2'b10: proc_rdata = block[block_index][95 :64];
			2'b01: proc_rdata = block[block_index][63 :32];
			2'b00: proc_rdata = block[block_index][31 :0 ];
		    endcase
		end
		else if(hit && valid && proc_write) begin
		    case(block_offset)
			2'b11: begin
			    block_nxt[block_index][127:96] = proc_wdata;
			    block_nxt[block_index][128] = 1;
			end
			2'b10: begin
			    block_nxt[block_index][95 :64] = proc_wdata;
			    block_nxt[block_index][128] = 1;
			end
			2'b01: begin
			    block_nxt[block_index][63 :32] = proc_wdata;
			    block_nxt[block_index][128] = 1;
			end
			2'b00: begin
			    block_nxt[block_index][31 :0 ] = proc_wdata;
			    block_nxt[block_index][128] = 1;
			end
		    endcase
		end
	    end

	    ALLOCATE: begin
		if(mem_ready_r) begin
		    block_nxt[block_index][154] = 1;
		    block_nxt[block_index][153:129] = proc_tag;
		    block_nxt[block_index][128] = 0;
		    block_nxt[block_index][127:0] = mem_rdata_r;
		end
	    end
	endcase
    end


//==== sequential circuit =================================

always@( posedge clk ) begin

    if( proc_reset ) begin
	mem_ready_r <= 1'b0;
	mem_rdata_r <= 128'b0;
	state  <= IDLE;
   	for(i = 0; i < 8; i = i + 1) 
	    block[i] <= 0; 
    end
    else begin 
        mem_ready_r <= mem_ready;
        state <= state_nxt;
        mem_rdata_r <= mem_rdata;
        for(i = 0; i < 8; i = i + 1)
	    block[i] <= block_nxt[i];
    end

end


endmodule

module cache_I(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    
//==== wire/reg definition ================================

    parameter IDLE      = 1'b0;
    parameter COMPARE   = 1'b0;
    parameter ALLOCATE  = 1'b1;

    parameter VALIDBIT  = 1;
    parameter TAGWIDTH  = 25;
    parameter DIRTYBIT  = 1;
    parameter DATAWIDTH = 128;

    integer i, j;

    reg         proc_stall, mem_read, mem_write;
    reg [27 :0] mem_addr;
    reg [127:0] mem_wdata;
    reg [31 :0] proc_rdata;

    reg [VALIDBIT + TAGWIDTH + DIRTYBIT + DATAWIDTH - 1 : 0] block [0:7];
    reg [VALIDBIT + TAGWIDTH + DIRTYBIT + DATAWIDTH - 1 : 0] block_nxt [0:7];
    
    reg  state;
    reg  state_nxt;


    reg [127:0]  mem_rdata_r;
    reg          mem_ready_r;
    reg         valid, hit;
    wire [1:0]  block_offset;
    wire [2:0]  block_index;
    wire [24:0] proc_tag;

//==== finite state machine  =============================

    always@(*) begin
	state_nxt = IDLE;
	case(state)
	    COMPARE: begin
		if (proc_read) begin
		    if(hit && valid)
		    	state_nxt = COMPARE;
		    else 
		    	state_nxt = ALLOCATE;
		end
		else
		    state_nxt = COMPARE;
	    end
	    ALLOCATE: begin
		if (!mem_ready_r)
		    state_nxt = ALLOCATE;
		else
		    state_nxt = COMPARE;
	    end
	endcase
    end

//==== combinational circuit ==============================

    assign block_offset = proc_addr[1:0];
    assign block_index = proc_addr[4:2];
    assign proc_tag = proc_addr[29:5];

    always@(*) begin/// control unit
	proc_stall = 0;
	valid = 0;
	hit = 0;
	mem_addr  = 28'h0000000;
	mem_read  = 0;
	mem_write = 0;
	mem_wdata = 0;
	case(state)
	    COMPARE: begin
		valid = block[block_index][154];
		hit   = (proc_tag == block[block_index][153:129]);
		if((hit && valid) || (!proc_read))
		    proc_stall = 0;
		else
		    proc_stall = 1;
	    end
	    ALLOCATE: begin
		proc_stall = 1'b1;
		mem_read  = (mem_ready_r) ? 1'b0 : 1'b1;
		mem_write = 0;
		mem_addr  = proc_addr[29:2];
	    end
	endcase
    end

    always@(*) begin/// data flow
	proc_rdata = 32'h0;
	for (i = 0; i < 8; i = i + 1) begin
	    block_nxt[i] = block[i];
	end
	case(state)
	    COMPARE: begin
		if(hit && valid && proc_read) begin
		    case(block_offset)
			2'b11: proc_rdata = block[block_index][127:96];
			2'b10: proc_rdata = block[block_index][95 :64];
			2'b01: proc_rdata = block[block_index][63 :32];
			2'b00: proc_rdata = block[block_index][31 :0 ];
		    endcase
		end
	    end

	    ALLOCATE: begin
		if(mem_ready_r) begin
		    block_nxt[block_index][154] = 1;
		    block_nxt[block_index][153:129] = proc_tag;
		    block_nxt[block_index][128] = 0;
		    block_nxt[block_index][127:0] = mem_rdata_r;
		end
	    end
	endcase
    end


//==== sequential circuit =================================

always@( posedge clk ) begin

    if( proc_reset ) begin
	mem_ready_r <= 1'b0;
	mem_rdata_r <= 128'b0;
	state  <= IDLE;
   	for(i = 0; i < 8; i = i + 1) 
	    block[i] <= 0; 
    end
    else begin 
        mem_ready_r <= mem_ready;
        state <= state_nxt;
        mem_rdata_r <= mem_rdata;
        for(i = 0; i < 8; i = i + 1)
	    block[i] <= block_nxt[i];
    end

end


endmodule
