`define MEM_MINWIDTH 1
`define MEM_MAXADDR PPP
`define MEM_MAXDATA 36
`define MAX(a,b) (a > b ? a : b)
`define MIN(a,b) (a < b ? a : b)

module \$mem (RD_CLK, RD_ADDR, RD_DATA, WR_CLK, WR_EN, WR_ADDR, WR_DATA);
	parameter MEMID = "";
	parameter SIZE = 256;
	parameter OFFSET = 0;
	parameter ABITS = 8;
	parameter WIDTH = 8;
	
	parameter RD_PORTS = 1;
	parameter RD_CLK_ENABLE = 1'b1;
	parameter RD_CLK_POLARITY = 1'b1;
	parameter RD_TRANSPARENT = 1'b1;
	
	parameter WR_PORTS = 1;
	parameter WR_CLK_ENABLE = 1'b1;
	parameter WR_CLK_POLARITY = 1'b1;
	
	input [RD_PORTS-1:0] RD_CLK;
	input [RD_PORTS*ABITS-1:0] RD_ADDR;
	output reg [RD_PORTS*WIDTH-1:0] RD_DATA;
	
	input [WR_PORTS-1:0] WR_CLK;
	input [WR_PORTS*ABITS-1:0] WR_ADDR;
	input [WR_PORTS*WIDTH-1:0] WR_DATA, WR_EN;
	
	wire [1023:0] _TECHMAP_DO_ = "proc; clean";
	
	parameter _TECHMAP_CONNMAP_RD_CLK_ = 0;
	parameter _TECHMAP_CONNMAP_WR_CLK_ = 0;
	parameter _TECHMAP_CONNMAP_RD_ADDR_ = 0;
	parameter _TECHMAP_CONNMAP_WR_ADDR_ = 0;
	parameter _TECHMAP_CONNMAP_WR_EN_ = 0;
	parameter _TECHMAP_BITS_CONNMAP_ = 0;
	//parameter _TECHMAP_CONNMAP_RD_PORTS_ = 0;
	//parameter _TECHMAP_CONNMAP_WR_PORTS_ = 0;
	
	reg _TECHMAP_FAIL_;
	initial begin
		_TECHMAP_FAIL_ <= 0;
	
		// only map cells with only one read and one write port
		if (RD_PORTS > 2 || WR_PORTS > 2)
			_TECHMAP_FAIL_ <= 1;
	
		// we expect positive read clock and non-transparent reads
		if (RD_TRANSPARENT || !RD_CLK_ENABLE || !RD_CLK_POLARITY)
			_TECHMAP_FAIL_ <= 1;
	
		// we expect positive write clock
		if (!WR_CLK_ENABLE || !WR_CLK_POLARITY)
			_TECHMAP_FAIL_ <= 1;
	
		// read and write must be in same clock domain
		if (_TECHMAP_CONNMAP_RD_CLK_ != _TECHMAP_CONNMAP_WR_CLK_)
			_TECHMAP_FAIL_ <= 1;
	
		// we don't do small memories or memories with offsets
		if (OFFSET != 0 || ABITS < `MEM_MINWIDTH || SIZE < 2**`MEM_MINWIDTH)
			_TECHMAP_FAIL_ <= 1;
	
	end

	genvar i;
	for (i = 0; i < `MAX(RD_PORTS, WR_PORTS); i = i+1) begin
		initial begin
			// check each pair of read and write port are the same
			if (RD_PORTS >= i && WR_PORTS >= i) begin
				if (_TECHMAP_CONNMAP_RD_ADDR_[ABITS*_TECHMAP_BITS_CONNMAP_*(i+1)-1:ABITS*_TECHMAP_BITS_CONNMAP_*i] != _TECHMAP_CONNMAP_WR_ADDR_[ABITS*_TECHMAP_BITS_CONNMAP_*(i+1)-1:ABITS*_TECHMAP_BITS_CONNMAP_*i])
					_TECHMAP_FAIL_ <= 1;
			end
		end
		// check all bits of write enable are the same
		if (i < WR_PORTS) begin
			genvar j;
			for (j = 1; j < WIDTH; j = j+1) begin
				initial begin
					if (_TECHMAP_CONNMAP_WR_EN_[(WIDTH*i+j+1)*_TECHMAP_BITS_CONNMAP_-1:(WIDTH*i+j)*_TECHMAP_BITS_CONNMAP_] != _TECHMAP_CONNMAP_WR_EN_[(WIDTH*i+1)*_TECHMAP_BITS_CONNMAP_-1:(WIDTH*i)*_TECHMAP_BITS_CONNMAP_])
						_TECHMAP_FAIL_ <= 1;
				end
			end
		end
	end

	
	\$__mem_gen #(
		.MEMID(MEMID), .SIZE(SIZE), .OFFSET(OFFSET), .ABITS(ABITS), .WIDTH(WIDTH),
		.RD_PORTS(RD_PORTS), .RD_CLK_ENABLE(RD_CLK_ENABLE), .RD_CLK_POLARITY(RD_CLK_POLARITY), .RD_TRANSPARENT(RD_TRANSPARENT),
		.WR_PORTS(WR_PORTS), .WR_CLK_ENABLE(WR_CLK_ENABLE), .WR_CLK_POLARITY(WR_CLK_POLARITY)
	) _TECHMAP_REPLACE_ (
		.RD_CLK(RD_CLK),
		.RD_ADDR(RD_ADDR),
		.RD_DATA(RD_DATA),
		.WR_CLK(WR_CLK),
		.WR_EN(WR_EN),
		.WR_ADDR(WR_ADDR),
		.WR_DATA(WR_DATA)
	);
endmodule

module \$__mem_gen (RD_CLK, RD_ADDR, RD_DATA, WR_CLK, WR_EN, WR_ADDR, WR_DATA);
	parameter MEMID = "";
	parameter SIZE = 256;
	parameter OFFSET = 0;
	parameter ABITS = 8;
	parameter WIDTH = 8;
	
	parameter RD_PORTS = 1;
	parameter RD_CLK_ENABLE = 1'b1;
	parameter RD_CLK_POLARITY = 1'b1;
	parameter RD_TRANSPARENT = 1'b1;
	
	parameter WR_PORTS = 1;
	parameter WR_CLK_ENABLE = 1'b1;
	parameter WR_CLK_POLARITY = 1'b1;
	
	input [RD_PORTS-1:0] RD_CLK;
	input [RD_PORTS*ABITS-1:0] RD_ADDR;
	output reg [RD_PORTS*WIDTH-1:0] RD_DATA;
	
	input [WR_PORTS-1:0] WR_CLK;
	input [WR_PORTS*ABITS-1:0] WR_ADDR;
	input [WR_PORTS*WIDTH-1:0] WR_DATA, WR_EN;

	wire [1023:0] _TECHMAP_DO_ = "proc; clean";

	genvar i;
	generate
		if (ABITS > `MEM_MAXADDR) begin
			wire [WIDTH-1:0] rd_data_hi, rd_data_lo;
			wire [(ABITS-1)*RD_PORTS-1:0] rd_addr_new;
			for (i = 0; i < RD_PORTS; i = i+1) begin
				assign rd_addr_new[(ABITS-1)*(i+1):(ABITS-1)*i] = RD_ADDR[ABITS*(i+1)-2:ABITS*i];
			end
			wire [(ABITS-1)*WR_PORTS-1:0] wr_addr_new;
			wire [WR_PORTS-1:0] wr_en_new;
			for (i = 0; i < WR_PORTS; i = i+1) begin
				assign wr_addr_new[(ABITS-1)*(i+1):(ABITS-1)*i] = WR_ADDR[ABITS*(i+1)-2:ABITS*i];
				assign wr_en_new[i] = WR_EN[i] & WR_ADDR[ABITS*(i+1)-1];
			end

			if (SIZE > 2**(ABITS-1)) begin
				\$__mem_gen #(
					.MEMID(MEMID), .SIZE(SIZE - 2**(ABITS-1)), .OFFSET(OFFSET), .ABITS(ABITS-1), .WIDTH(WIDTH),
					.RD_PORTS(RD_PORTS), .RD_CLK_ENABLE(RD_CLK_ENABLE), .RD_CLK_POLARITY(RD_CLK_POLARITY), .RD_TRANSPARENT(RD_TRANSPARENT),
					.WR_PORTS(WR_PORTS), .WR_CLK_ENABLE(WR_CLK_ENABLE), .WR_CLK_POLARITY(WR_CLK_POLARITY)
				) mem_hi (
					.RD_CLK(RD_CLK),
					.RD_ADDR(rd_addr_new),
					.RD_DATA(rd_data_hi),
					.WR_CLK(WR_CLK),
					.WR_EN(wr_en_new),
					.WR_ADDR(wr_addr_new),
					.WR_DATA(WR_DATA)
				);
			end 
			else begin
				assign rd_data_hi = {{WIDTH}{1'bx}};
			end

			\$__mem_gen #(
				.MEMID(MEMID), .SIZE(SIZE > 2**(ABITS-1) ? 2**(ABITS-1) : SIZE), .OFFSET(OFFSET), .ABITS(ABITS-1), .WIDTH(WIDTH),
				.RD_PORTS(RD_PORTS), .RD_CLK_ENABLE(RD_CLK_ENABLE), .RD_CLK_POLARITY(RD_CLK_POLARITY), .RD_TRANSPARENT(RD_TRANSPARENT),
				.WR_PORTS(WR_PORTS), .WR_CLK_ENABLE(WR_CLK_ENABLE), .WR_CLK_POLARITY(WR_CLK_POLARITY)
			) mem_lo (
				.RD_CLK(RD_CLK),
				.RD_ADDR(rd_addr_new),
				.RD_DATA(rd_data_lo),
				.WR_CLK(WR_CLK),
				.WR_EN(wr_en_new),
				.WR_ADDR(wr_addr_new),
				.WR_DATA(WR_DATA)
			);

			reg [RD_PORTS-1:0] delayed_abit;
			for (i = 0; i < RD_PORTS; i = i+1) begin
				always @(posedge RD_CLK[i])
					delayed_abit[i] <= RD_ADDR[ABITS*(i+1)-1];
				assign RD_DATA[WIDTH*(i+1)-1:WIDTH*i] = delayed_abit[i] ? rd_data_hi : rd_data_lo;
			end
		end 
		else begin
			localparam step = (	ABITS == 15 ? 1 :
						ABITS == 14 ? 2 :
						ABITS == 13 ? 4 :
						ABITS == 12 ? 9 :
						ABITS == 11 ? 18 :
						ABITS == 10 ? 36 :
						/*ABITS == 9 ?*/ 
						RD_PORTS < 2 && WR_PORTS < 2 ? 72 : 36);
			for (i = 0; i < WIDTH; i = i+step) begin:slice
				wire [`MEM_MAXDATA-1:0] in1, in2;
				wire [`MEM_MAXDATA-1:0] out1, out2;
				wire [ABITS-1:0] addr2;
				wire [7:0] we2;

				assign in1 = `MEM_MAXDATA'bx;
				assign in2 = `MEM_MAXDATA'bx;
				case (step)
					/* Twiddle data so that it still works
					* even if we drop to RAMB18 */
					9: assign {in1[8-1:8/2], in1[32], in1[8/2-1:0]} = WR_DATA[`MIN(i+step,WIDTH)-1:i];
					18: assign {in1[33], in1[16-1:16/2], in1[32], in1[16/2-1:0]} = WR_DATA[`MIN(i+step,WIDTH)-1:i];
					36: assign {in1[35:34], in1[32-1:32/2], in1[33:32], in1[32/2-1:0]} = WR_DATA[`MIN(i+step,WIDTH)-1:i];
					72: assign {in2[35:34], in1[35:34], in2[64/2-1:64/4], in1[64/2-1:64/4], 
						    in2[33:32], in1[33:32], in2[64/4-1:0], in1[64/4-1:0]} = WR_DATA[`MIN(i+step,WIDTH)-1:i];
					default: assign in1[step-1:0] = WR_DATA[`MIN(i+step,WIDTH)-1:i];
				endcase
				if (WR_PORTS > 1) begin
					case (step)
						9: assign {in2[8-1:8/2], in2[32], in2[8/2-1:0]} = WR_DATA[WIDTH+`MIN(i+step,WIDTH)-1:WIDTH+i];
						18: assign {in2[33], in2[16-1:16/2], in2[32], in2[16/2-1:0]} = WR_DATA[WIDTH+`MIN(i+step,WIDTH)-1:WIDTH+i];
						36: assign {in2[35:34], in2[32-1:32/2], in2[33:32], in2[32/2-1:0]} = WR_DATA[WIDTH+`MIN(i+step,WIDTH)-1:WIDTH+i];
						default:  assign in2[step-1:0] = WR_DATA[WIDTH+`MIN(i+step,WIDTH)-1:WIDTH+i];
					endcase
				end

				case (step)
					9: assign RD_DATA[`MIN(i+step,WIDTH)-1:i] = { out1[8-1:8/2], out1[32], out1[8/2-1:0] };
					18: assign RD_DATA[`MIN(i+step,WIDTH)-1:i] = { out1[33], out1[16-1:16/2], out1[32], out1[16/2-1:0] };
					36: assign RD_DATA[`MIN(i+step,WIDTH)-1:i] = { out1[35:34], out1[32-1:32/2], out1[33:32], out1[32/2-1:0] };
					72: assign RD_DATA[`MIN(i+step,WIDTH)-1:i] = { 
						out2[35:34], out1[35:34], out2[64/2-1:64/4], out1[64/2-1:64/4], 
						out2[33:32], out1[33:32], out2[64/4-1:0], out1[64/4-1:0] };
					default: assign RD_DATA[`MIN(i+step,WIDTH)-1:i] = out1[step-1:0];
				endcase
				if (RD_PORTS > 1) begin
					case (step)
						9: assign RD_DATA[WIDTH+`MIN(i+step,WIDTH)-1:WIDTH+i] = { out2[8-1:8/2], out2[32], out2[8/2-1:0] };
						18: assign RD_DATA[WIDTH+`MIN(i+step,WIDTH)-1:WIDTH+i] = { out2[33], out2[16-1:16/2], out2[32], out2[16/2-1:0] };
						36: assign RD_DATA[WIDTH+`MIN(i+step,WIDTH)-1:WIDTH+i] = { out2[35:34], out2[32-1:32/2], out2[33:32], out2[32/2-1:0] };
						default: assign RD_DATA[WIDTH+`MIN(i+step,WIDTH)-1:WIDTH+i] = out2[step-1:0];
					endcase
				end

				if (step < 72) begin
					if (RD_PORTS > 1 || WR_PORTS > 1) begin
						assign addr2 = RD_ADDR[2*ABITS-1:ABITS];
						assign we2 = {{8}{WR_EN[WIDTH]}};
					end
					else begin
						assign addr2 = `MEM_MAXADDR'bx;
						assign we2[7:0] = 8'b0;
					end
				end
				else begin
					assign addr2 = RD_ADDR;
					assign we2[7:0] = {{8}{WR_EN[0]}};
				end

				if (WIDTH <= i + step/2) begin
					RAMB18E1 #(
						// For single port, step >= 36, use `MEM_MAXDATA and both ports
						.READ_WIDTH_A(`MIN(step, `MEM_MAXDATA)/2),
						.READ_WIDTH_B(`MIN(step, `MEM_MAXDATA)/2),
						.WRITE_WIDTH_A(`MIN(step, `MEM_MAXDATA)/2),
						.WRITE_WIDTH_B(`MIN(step, `MEM_MAXDATA)/2)
					) m (
						.CLKARDCLK(RD_CLK[0]),
						.ADDRARDADDR0(RD_ADDR[0]), .ADDRARDADDR1(RD_ADDR[1]), .ADDRARDADDR2(RD_ADDR[2]), .ADDRARDADDR3(RD_ADDR[3]), .ADDRARDADDR4(RD_ADDR[4]), 
						.ADDRARDADDR5(RD_ADDR[5]), .ADDRARDADDR6(RD_ADDR[6]), .ADDRARDADDR7(RD_ADDR[7]), .ADDRARDADDR8(RD_ADDR[8]),  .ADDRARDADDR9(RD_ADDR[9]),
						.ADDRARDADDR10(RD_ADDR[10]), .ADDRARDADDR11(RD_ADDR[11]), .ADDRARDADDR12(RD_ADDR[12]), .ADDRARDADDR13(RD_ADDR[13]),
						.DOADO0(out1[0]), .DOADO1(out1[1]), .DOADO2(out1[2]), .DOADO3(out1[3]), .DOADO4(out1[4]), .DOADO5(out1[5]), .DOADO6(out1[6]), .DOADO7(out1[7]), .DOADO8(out1[8]), .DOADO9(out1[9]),
						.DOADO10(out1[10]), .DOADO11(out1[11]), .DOADO12(out1[12]), .DOADO13(out1[13]), .DOADO14(out1[14]), .DOADO15(out1[15]),
						.DOPADOP0(out1[32]),  .DOPADOP1(out1[33]),
						.DIADI0(in1[0]), .DIADI1(in1[1]), .DIADI2(in1[2]), .DIADI3(in1[3]), .DIADI4(in1[4]), .DIADI5(in1[5]), .DIADI6(in1[6]),  .DIADI7(in1[7]), .DIADI8(in1[8]), .DIADI9(in1[9]),
						.DIADI10(in1[10]), .DIADI11(in1[11]), .DIADI12(in1[12]), .DIADI13(in1[13]), .DIADI14(in1[14]), .DIADI15(in1[15]),
						.DIPADIP0(in1[32]), .DIPADIP1(in1[33]),
						.WEA0(WR_EN[0]), .WEA1(WR_EN[0]),
						.CLKBWRCLK(RD_CLK[1]),
						.ADDRBWRADDR0(addr2[0]), .ADDRBWRADDR1(addr2[1]), .ADDRBWRADDR2(addr2[2]), .ADDRBWRADDR3(addr2[3]), .ADDRBWRADDR4(addr2[4]),
						.ADDRBWRADDR5(addr2[5]), .ADDRBWRADDR6(addr2[6]), .ADDRBWRADDR7(addr2[7]), .ADDRBWRADDR8(addr2[8]), .ADDRBWRADDR9(addr2[9]),
						.ADDRBWRADDR10(addr2[10]), .ADDRBWRADDR11(addr2[11]), .ADDRBWRADDR12(addr2[12]), .ADDRBWRADDR13(addr2[13]),
						.DOBDO0(out2[0]), .DOBDO1(out2[1]), .DOBDO2(out2[2]), .DOBDO3(out2[3]), .DOBDO4(out2[4]), .DOBDO5(out2[5]), .DOBDO6(out2[6]), .DOBDO7(out2[7]), .DOBDO8(out2[8]), .DOBDO9(out2[9]),
						.DOBDO10(out2[10]), .DOBDO11(out2[11]), .DOBDO12(out2[12]), .DOBDO13(out2[13]), .DOBDO14(out2[14]), .DOBDO15(out2[15]),
						.DOPBDOP0(out2[32]),  .DOPBDOP1(out2[33]),
						.DIBDI0(in2[0]), .DIBDI1(in2[1]), .DIBDI2(in2[2]), .DIBDI3(in2[3]), .DIBDI4(in2[4]), .DIBDI5(in2[5]), .DIBDI6(in2[6]),  .DIBDI7(in2[7]), .DIBDI8(in2[8]), .DIBDI9(in2[9]),
						.DIBDI10(in2[10]), .DIBDI11(in2[11]), .DIBDI12(in2[12]), .DIBDI13(in2[13]), .DIBDI14(in2[14]), .DIBDI15(in2[15]),
						.DIPBDIP0(in2[32]), .DIPBDIP1(in2[33]),
						.WEBWE0(we2[0]), .WEBWE1(we2[1]), .WEBWE2(we2[2]), .WEBWE3(we2[3])
					);
				end
				else begin
					RAMB36E1 #(
						.step(step),
						.rd_ports(RD_PORTS),
						// For single port, step >= 72, use `MEM_MAXDATA and both ports
						.READ_WIDTH_A(`MIN(step, `MEM_MAXDATA)),
						.READ_WIDTH_B(`MIN(step, `MEM_MAXDATA)),
						.WRITE_WIDTH_A(`MIN(step, `MEM_MAXDATA)),
						.WRITE_WIDTH_B(`MIN(step, `MEM_MAXDATA))
					) m (
						.CLKARDCLK(RD_CLK[0]),
						.ADDRARDADDR0(RD_ADDR[0]), .ADDRARDADDR1(RD_ADDR[1]), .ADDRARDADDR2(RD_ADDR[2]), .ADDRARDADDR3(RD_ADDR[3]), .ADDRARDADDR4(RD_ADDR[4]), 
						.ADDRARDADDR5(RD_ADDR[5]), .ADDRARDADDR6(RD_ADDR[6]), .ADDRARDADDR7(RD_ADDR[7]), .ADDRARDADDR8(RD_ADDR[8]),  .ADDRARDADDR9(RD_ADDR[9]),
						.ADDRARDADDR10(RD_ADDR[10]), .ADDRARDADDR11(RD_ADDR[11]), .ADDRARDADDR12(RD_ADDR[12]), .ADDRARDADDR13(RD_ADDR[13]), .ADDRARDADDR14(RD_ADDR[14]),
						.DOADO0(out1[0]), .DOADO1(out1[1]), .DOADO2(out1[2]), .DOADO3(out1[3]), .DOADO4(out1[4]), .DOADO5(out1[5]), .DOADO6(out1[6]), .DOADO7(out1[7]), .DOADO8(out1[8]), .DOADO9(out1[9]),
						.DOADO10(out1[10]), .DOADO11(out1[11]), .DOADO12(out1[12]), .DOADO13(out1[13]), .DOADO14(out1[14]), .DOADO15(out1[15]), .DOADO16(out1[16]), .DOADO17(out1[17]), .DOADO18(out1[18]), .DOADO19(out1[19]),
						.DOADO20(out1[20]), .DOADO21(out1[21]), .DOADO22(out1[22]), .DOADO23(out1[23]), .DOADO24(out1[24]), .DOADO25(out1[25]), .DOADO26(out1[26]), .DOADO27(out1[27]), .DOADO28(out1[28]), .DOADO29(out1[29]),
						.DOADO30(out1[30]), .DOADO31(out1[31]),
						.DOPADOP0(out1[32]),  .DOPADOP1(out1[33]), .DOPADOP2(out1[34]), .DOPADOP3(out1[35]),
						.DIADI0(in1[0]), .DIADI1(in1[1]), .DIADI2(in1[2]), .DIADI3(in1[3]), .DIADI4(in1[4]), .DIADI5(in1[5]), .DIADI6(in1[6]),  .DIADI7(in1[7]), .DIADI8(in1[8]), .DIADI9(in1[9]),
						.DIADI10(in1[10]), .DIADI11(in1[11]), .DIADI12(in1[12]), .DIADI13(in1[13]), .DIADI14(in1[14]), .DIADI15(in1[15]), .DIADI16(in1[16]),  .DIADI17(in1[17]), .DIADI18(in1[18]), .DIADI19(in1[19]),
						.DIADI20(in1[20]), .DIADI21(in1[21]), .DIADI22(in1[22]), .DIADI23(in1[23]), .DIADI24(in1[24]), .DIADI25(in1[25]), .DIADI26(in1[26]),  .DIADI27(in1[27]), .DIADI28(in1[28]), .DIADI29(in1[29]),
						.DIADI30(in1[30]), .DIADI31(in1[31]), 
						.DIPADIP0(in1[32]), .DIPADIP1(in1[33]), .DIPADIP2(in1[34]), .DIPADIP3(in1[35]),
						.WEA0(WR_EN[0]), .WEA1(WR_EN[0]), .WEA2(WR_EN[0]),.WEA3(WR_EN[0]),
						.CLKBWRCLK(RD_CLK[1]),
						.ADDRBWRADDR0(addr2[0]), .ADDRBWRADDR1(addr2[1]), .ADDRBWRADDR2(addr2[2]), .ADDRBWRADDR3(addr2[3]), .ADDRBWRADDR4(addr2[4]),
						.ADDRBWRADDR5(addr2[5]), .ADDRBWRADDR6(addr2[6]), .ADDRBWRADDR7(addr2[7]), .ADDRBWRADDR8(addr2[8]), .ADDRBWRADDR9(addr2[9]),
						.ADDRBWRADDR10(addr2[10]), .ADDRBWRADDR11(addr2[11]), .ADDRBWRADDR12(addr2[12]), .ADDRBWRADDR13(addr2[13]), .ADDRBWRADDR14(addr2[14]),
						.DOBDO0(out2[0]), .DOBDO1(out2[1]), .DOBDO2(out2[2]), .DOBDO3(out2[3]), .DOBDO4(out2[4]), .DOBDO5(out2[5]), .DOBDO6(out2[6]), .DOBDO7(out2[7]), .DOBDO8(out2[8]), .DOBDO9(out2[9]),
						.DOBDO10(out2[10]), .DOBDO11(out2[11]), .DOBDO12(out2[12]), .DOBDO13(out2[13]), .DOBDO14(out2[14]), .DOBDO15(out2[15]), .DOBDO16(out2[16]), .DOBDO17(out2[17]), .DOBDO18(out2[18]), .DOBDO19(out2[19]),
						.DOBDO20(out2[20]), .DOBDO21(out2[21]), .DOBDO22(out2[22]), .DOBDO23(out2[23]), .DOBDO24(out2[24]), .DOBDO25(out2[25]), .DOBDO26(out2[26]), .DOBDO27(out2[27]), .DOBDO28(out2[28]), .DOBDO29(out2[29]),
						.DOBDO30(out2[30]), .DOBDO31(out2[31]),
						.DOPBDOP0(out2[32]),  .DOPBDOP1(out2[33]), .DOPBDOP2(out2[34]), .DOPBDOP3(out2[35]),
						.DIBDI0(in2[0]), .DIBDI1(in2[1]), .DIBDI2(in2[2]), .DIBDI3(in2[3]), .DIBDI4(in2[4]), .DIBDI5(in2[5]), .DIBDI6(in2[6]),  .DIBDI7(in2[7]), .DIBDI8(in2[8]), .DIBDI9(in2[9]),
						.DIBDI10(in2[10]), .DIBDI11(in2[11]), .DIBDI12(in2[12]), .DIBDI13(in2[13]), .DIBDI14(in2[14]), .DIBDI15(in2[15]), .DIBDI16(in2[16]),  .DIBDI17(in2[17]), .DIBDI18(in2[18]), .DIBDI19(in2[19]),
						.DIBDI20(in2[20]), .DIBDI21(in2[21]), .DIBDI22(in2[22]), .DIBDI23(in2[23]), .DIBDI24(in2[24]), .DIBDI25(in2[25]), .DIBDI26(in2[26]),  .DIBDI27(in2[27]), .DIBDI28(in2[28]), .DIBDI29(in2[29]),
						.DIBDI30(in2[30]), .DIBDI31(in2[31]), 
						.DIPBDIP0(in2[32]), .DIPBDIP1(in2[33]), .DIPBDIP2(in2[34]), .DIPBDIP3(in2[35]),
						.WEBWE0(we2[0]), .WEBWE1(we2[1]), .WEBWE2(we2[2]), .WEBWE3(we2[3]), .WEBWE4(we2[4]), .WEBWE5(we2[5]), .WEBWE6(we2[6]), .WEBWE7(we2[7]) 
					);
				end
			end
		end
	endgenerate
endmodule

module multiply(a, b, p);
parameter A_WIDTH = 25;
parameter B_WIDTH = 18;
parameter Y_WIDTH = A_WIDTH+B_WIDTH;
input [A_WIDTH-1:0] a;
input [B_WIDTH-1:0] b;
output [Y_WIDTH-1:0] p;

DSP48E1 #(
	.AREG(0), 
	.BREG(0), 
	.PREG(0),
	.MREG(0),
	.DREG(0),
	.ADREG(0),
	.ACASCREG(0),
	.BCASCREG(0),
	.USE_DPORT("FALSE"),
	.USE_MULT("MULTIPLY")
) 
_TECHMAP_REPLACE_ 
(
	.CEA1(1'b0), .CEA2(1'b0), .CEB1(1'b0), .CEB2(1'b0), .CEM(1'b0), .CEP(1'b0),
	// {X,Y} muxes from M
	.OPMODE0 (1'b1), .OPMODE2 (1'b1),
	.A0(a[ 0]), .A1(a[ 1]), .A2(a[ 2]), .A3(a[ 3]), .A4(a[ 4]), .A5(a[ 5]), .A6(a[ 6]), .A7(a[ 7]), .A8(a[ 8]), .A9(a[ 9]),
	.A10(a[10]), .A11(a[11]), .A12(a[12]), .A13(a[13]), .A14(a[14]), .A15(a[15]), .A16(a[16]), .A17(a[17]), .A18(a[18]), .A19(a[19]), .A20(a[20]),
	.A21(a[21]), .A22(a[22]), .A23(a[23]), .A24(a[24]),
	.B0(b[ 0]), .B1(b[ 1]), .B2(b[ 2]), .B3(b[ 3]), .B4(b[ 4]), .B5(b[ 5]), .B6(b[ 6]), .B7(b[ 7]), .B8(b[ 8]), .B9(b[ 9]),
	.B10(b[10]), .B11(b[11]), .B12(b[12]), .B13(b[13]), .B14(b[14]), .B15(b[15]),.B16(b[16]), .B17(b[17]),
	.P0(p[ 0]), .P1(p[ 1]), .P2(p[ 2]), .P3(p[ 3]), .P4(p[ 4]), .P5(p[ 5]), .P6(p[ 6]), .P7(p[ 7]), .P8(p[ 8]), .P9(p[ 9]),
	.P10(p[10]), .P11(p[11]), .P12(p[12]), .P13(p[13]), .P14(p[14]), .P15(p[15]), .P16(p[16]), .P17(p[17]), .P18(p[18]), .P19(p[19]),
	.P20(p[20]), .P21(p[21]), .P22(p[22]), .P23(p[23]), .P24(p[24]), .P25(p[25]), .P26(p[26]), .P27(p[27]), .P28(p[28]), .P29(p[29]),
	.P30(p[30]), .P31(p[31]), .P32(p[32]), .P33(p[33]), .P34(p[34]), .P35(p[35]), .P36(p[36]), .P37(p[37]), .P38(p[38]), .P39(p[39]),
	.P40(p[40]), .P41(p[41]), .P42(p[42])
);

endmodule

module xadder(a_xor_b, a_and_b, cin, cout, sumout);
parameter _TECHMAP_CONSTMSK_cin_ = 0;
parameter _TECHMAP_CONSTVAL_cin_ = 0;

input a_xor_b, a_and_b, cin;
output cout, sumout;

wire [1023:0] _TECHMAP_DO_ = "proc; clean";
// If cin is constant at 1'bx, then set it to 1'b0
// (map errors out otherwise)
wire cin0 = _TECHMAP_CONSTMSK_cin_ === 1'b1 && _TECHMAP_CONSTVAL_cin_ === 1'bx ? 1'b0 : cin;

MUXCY _cy (.S(a_xor_b), .DI(a_and_b), .CI(cin0), .O(cout));
XORCY _xor (.LI(a_xor_b), .CI(cin0), .O(sumout));

endmodule

module \$lut (A, Y);

  parameter WIDTH = 0;
  parameter LUT = 0;

  input [WIDTH-1:0] A;
  output Y;

  generate
    if (WIDTH == 8) begin:lut8
      wire [6-1:0] _y;
      (* U_SET="_TECHMAP_REPLACE_", RLOC="X0Y0", BEL="D6LUT" *)
      LUT6 #(.INIT(LUT[1*(2**(WIDTH-2))-1:0*(2**(WIDTH-2))])) fpga_lut_lo_lo (.I0(A[0]), .I1(A[1]), .I2(A[2]), .I3(A[3]), .I4(A[4]), .I5(A[5]), .O(_y[0]));
      (* U_SET="_TECHMAP_REPLACE_", RLOC="X0Y0", BEL="C6LUT" *)
      LUT6 #(.INIT(LUT[2*(2**(WIDTH-2))-1:1*(2**(WIDTH-2))])) fpga_lut_lo_hi (.I0(A[0]), .I1(A[1]), .I2(A[2]), .I3(A[3]), .I4(A[4]), .I5(A[5]), .O(_y[1]));
      (* U_SET="_TECHMAP_REPLACE_", RLOC="X0Y0", BEL="F7BMUX" *)
      MUXF7 fpga_lut_muxf7_lo (.O(_y[4]), .I0(_y[0]), .I1(_y[1]), .S(A[WIDTH-2]));

      (* U_SET="_TECHMAP_REPLACE_", RLOC="X0Y0", BEL="B6LUT" *)
      LUT6 #(.INIT(LUT[3*(2**(WIDTH-2))-1:2*(2**(WIDTH-2))])) fpga_lut_hi_lo (.I0(A[0]), .I1(A[1]), .I2(A[2]), .I3(A[3]), .I4(A[4]), .I5(A[5]), .O(_y[2]));
      (* U_SET="_TECHMAP_REPLACE_", RLOC="X0Y0", BEL="A6LUT" *)
      LUT6 #(.INIT(LUT[4*(2**(WIDTH-2))-1:3*(2**(WIDTH-2))])) fpga_lut_hi_hi (.I0(A[0]), .I1(A[1]), .I2(A[2]), .I3(A[3]), .I4(A[4]), .I5(A[5]), .O(_y[3]));
      (* U_SET="_TECHMAP_REPLACE_", RLOC="X0Y0", BEL="F7AMUX" *)
      MUXF7 fpga_lut_muxf7_hi (.O(_y[5]), .I0(_y[2]), .I1(_y[3]), .S(A[WIDTH-2]));

      (* U_SET="_TECHMAP_REPLACE_", RLOC="X0Y0" *)
      MUXF8 fpga_lut_muxf8 (.O(Y), .I0(_y[4]), .I1(_y[5]), .S(A[WIDTH-1]));
    end else
    if (WIDTH == 7) begin:lut7
      wire [2-1:0] _y;
      (* U_SET="_TECHMAP_REPLACE_", RLOC="X0Y0" *)
      LUT6 #(.INIT(LUT[1*(2**(WIDTH-1))-1:0*(2**(WIDTH-1))])) fpga_lut_lo (.I0(A[0]), .I1(A[1]), .I2(A[2]), .I3(A[3]), .I4(A[4]), .I5(A[5]), .O(_y[0]));
      (* U_SET="_TECHMAP_REPLACE_", RLOC="X0Y0" *)
      LUT6 #(.INIT(LUT[2*(2**(WIDTH-1))-1:1*(2**(WIDTH-1))])) fpga_lut_hi (.I0(A[0]), .I1(A[1]), .I2(A[2]), .I3(A[3]), .I4(A[4]), .I5(A[5]), .O(_y[1]));
      (* U_SET="_TECHMAP_REPLACE_", RLOC="X0Y0" *)
      MUXF7 fpga_lut_muxf7 (.O(Y), .I0(_y[0]), .I1(_y[1]), .S(A[WIDTH-1]));
    end else begin
      wire _TECHMAP_FAIL_ = 1;
    end
  endgenerate
endmodule


module bufgctrl(i, s, ce, ignore, o);
input [1:0] i;
input [1:0] s;
input [1:0] ce;
input [1:0] ignore;
output o;

BUFGCTRL _TECHMAP_REPLACE_ (.I0(i[0]), .I1(i[1]),
	.S0(s[0]), .S1(s[1]),
	.CE0(ce[0]), .CE1(ce[1]),
	.IGNORE0(ignore[0]), .IGNORE1(ignore[1]),
	.O(o));

endmodule
