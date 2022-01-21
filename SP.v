module SP(
	// INPUT SIGNAL
	clk,
	rst_n,
	in_valid,
	inst,
	mem_dout,
	// OUTPUT SIGNAL
	out_valid,
	inst_addr,
	mem_wen,
	mem_addr,
	mem_din
);

//------------------------------------------------------------------------
//   INPUT AND OUTPUT DECLARATION                         
//------------------------------------------------------------------------

input                    clk, rst_n, in_valid;
input             [31:0] inst;
input  signed     [31:0] mem_dout;
output reg               out_valid;
output reg        [31:0] inst_addr;
output reg               mem_wen;
output reg        [11:0] mem_addr;
output reg signed [31:0] mem_din;

//------------------------------------------------------------------------
//   DECLARATION
//------------------------------------------------------------------------

// REGISTER FILE, DO NOT EDIT THE NAME.
reg	        [31:0] r      [0:31]; 
reg 		[5:0]  opcode;
reg 		[4:0]  rs, rt, rd, shamt;
reg 		[5:0]  funct;
reg signed  [15:0] imm;

wire reg_dst, reg_write, alu_scr, pc_src, mem_read, mem_write, mem_to_reg;
assign reg_dst = (opcode == 'h0);
assign reg_write = (opcode <= 'h5);
assign alu_scr = !((opcode == 'h0) && (funct != 'h05) || (opcode == 'h7) || (opcode == 'h8));
assign pc_src = (opcode == 'h7) || (opcode == 'h8);
assign mem_read = (opcode == 'h5);
assign mem_write = (opcode == 'h6);
assign mem_to_reg = (opcode == 'h5);

wire [31:0] write_reg_addr;

MUX reg_dst_mux(
	.in0({27'b0, rt}),
	.in1({27'b0, rd}),
	.select(reg_dst),
	.out(write_reg_addr)
);

wire [31:0] extend_reslut;
wire [1:0] extend_ctrl;
assign extend_ctrl[1] = (opcode == 'h0) && (funct == 'h5);
assign extend_ctrl[0] = (opcode == 'h3) || (opcode == 'h4);

EXTEND_CONTROL extend_control(
	.imm(imm),
	.select(extend_ctrl),
	.out(extend_reslut)
);

reg [31:0] read_reg_value1, read_reg_value2, write_reg_addr1;

wire [31:0] alu_scr_value;

MUX alu_src_mux(
	.in0(read_reg_value2),
	.in1(extend_reslut),
	.select(alu_scr),
	.out(alu_scr_value)
);

wire [3:0] alu_ctrl;

ALU_CONTROL alu_control(
	.opcode(opcode),
	.funct(funct),
	.alu_ctrl(alu_ctrl)
);

wire        alu_zero;
wire [31:0] alu_result;

ALU alu(
	.in0(r[rs]),
	.in1(alu_scr_value),
	.op(alu_ctrl),
	.out(alu_result),
	.zero(alu_zero)
);

wire [31:0] pc_plus_four;

ADDER pc_adder(
	.in0(inst_addr),
	.in1('d4),
	.out(pc_plus_four)
);

wire [31:0] branched_pc;

ADDER branch_pc_adder(
	.in0(pc_plus_four),
	.in1({extend_reslut[29:0], 2'b0}),
	.out(branched_pc)
);

wire [31:0] next_pc;

MUX pc_mux(
	.in0(pc_plus_four),
	.in1(branched_pc),
	.select(pc_src & (opcode == 'h08 ? !alu_zero : alu_zero)),
	.out(next_pc)
);

wire [31:0] write_back;

MUX write_back_mux(
	.in0(alu_result),
	.in1(mem_dout),
	.select(mem_to_reg),
	.out(write_back)
);

localparam S_IDLE = 'd0;
localparam S_ID   = 'd1;
localparam S_REG  = 'd2;
localparam S_EXE  = 'd3;
localparam S_MEM1 = 'd4;
localparam S_MEM2 = 'd5;
localparam S_MEM3 = 'd6;
localparam S_WB   = 'd8;
localparam S_OUT  = 'd9;

reg [3:0] current_state, next_state;

//------------------------------------------------------------------------
//   DESIGN
//------------------------------------------------------------------------
integer i;
always @(posedge clk, negedge rst_n) begin
	if(!rst_n) begin
		current_state <= S_IDLE;
		inst_addr <= 32'd0;
		mem_addr <= 32'b0;
		mem_din <= 32'b0;
		mem_wen <= 1'b1;
		for(i = 0; i < 32; i = i + 1) begin
			r[i] <= 32'b0;
		end
	end	else begin
		current_state <= next_state;
	end
end

always @(*) begin
	case(current_state)
		S_IDLE : if (in_valid) next_state = S_ID;
		         else          next_state = S_IDLE;
		S_ID   : if (in_valid) next_state = S_ID;
		         else          next_state = S_REG;
		S_REG  : next_state = S_EXE;
		S_EXE  : next_state = S_MEM1;
		S_MEM1 : next_state = S_MEM2;
		S_MEM2 : next_state = S_MEM3;
		S_MEM3 : next_state = S_WB;
		S_WB   : next_state = S_OUT;
		S_OUT  : next_state = S_IDLE;
		default: next_state = S_IDLE;
	endcase
end

always @(current_state) begin
	case(current_state)
		S_IDLE : begin
			out_valid <= 1'b0;
		end
		S_ID   : begin
			opcode <= inst[31:26];
			rs     <= inst[25:21];
			rt     <= inst[20:16];
			rd     <= inst[15:11];
			shamt  <= inst[10:6];
			funct  <= inst[5:0];
			imm    <= inst[15:0];
		end
		S_REG  : begin
			read_reg_value1 <= r[rs];
			read_reg_value2 <= r[rt];
			write_reg_addr1 <= write_reg_addr;
		end
		S_EXE  : begin
			mem_wen  <= !mem_write;
			mem_addr <= alu_result;
		end
		S_MEM1 : begin
			mem_din <= r[rt];
		end
		S_WB   : begin
			if (reg_write) r[write_reg_addr1] <= write_back;
		end
		S_OUT  : begin
			inst_addr <= next_pc;
			out_valid <= 1'b1;
		end
		default: begin
		end
	endcase
end

endmodule

module MUX (
	in0,
	in1,
	select,
	out
);

input         select;
input  [31:0] in0, in1;
output [31:0] out;

assign out = select ? in1 : in0;
	
endmodule

module EXTEND_CONTROL (
	imm,
	select,
	out
);

input      [15:0] imm;
input      [1:0]  select;
output reg [31:0] out;

always @(*) begin
	case (select)
		2'b00: // zero-extend
			out <= {16'b0, imm};
		2'b01: // sign-extend
			out <= {{16{imm[15]}}, imm};
		2'b10: // shamt
			out <= {16'b0, 11'b0, imm[10:6]};
		default : 
			out <= 32'b0;
	endcase
end

endmodule

module ADDER (
	in0,
	in1,
	out
);

input      [31:0] in0, in1;
output reg [31:0] out;

always @(*) begin
	out <= in0 + in1;
end
	
endmodule

module ALU (
	in0,
	in1,
	op,
	out,
	zero
);

input signed      [31:0] in0, in1;
input             [3:0]  op;
output reg signed [31:0] out;
output reg               zero;
	
always @(*) begin
	case(op)
		'h0: // and
			out <= in0 & in1;
		'h1: // or
			out <= in0 | in1;
		'h2: // add
			out <= in0 + in1;
		'h3: // sub	
			out <= in0 - in1;
		'h4: // slt
			out <= in0 < in1;
		'h5: // sll
			out <= in0 << in1;
	endcase

	zero <= (out == 0);
end

endmodule

module ALU_CONTROL (
	opcode,
	funct,
	alu_ctrl
);

input [5:0] opcode;
input [5:0] funct;
output reg [3:0] alu_ctrl;

always @(*) begin
	case (opcode)
		'h0: // r-type
			alu_ctrl <= funct[3:0];
		'h1: // andi
			alu_ctrl <= 'h0;
		'h2: // ori
			alu_ctrl <= 'h1;
		'h3: // addi
			alu_ctrl <= 'h2;
		'h4: // subi
			alu_ctrl <= 'h3;
		'h5: // lw
			alu_ctrl <= 'h2;
		'h6: // sw
			alu_ctrl <= 'h2;
		'h7: // beq
			alu_ctrl <= 'h3;
		'h8: // bne
			alu_ctrl <= 'h3;
	endcase
end

endmodule