


`include "/usr/cad/synopsys/synthesis/2025.06/dw/sim_ver/DW_sqrt.v"
module REFRACT(
    input  wire        CLK,
    input  wire        RST,
    input  wire [3:0]  RI,   
    output reg  [8:0]  SRAM_A,
    output reg  [15:0] SRAM_D,
    input  wire [15:0] SRAM_Q,   // unused
    output reg         SRAM_WE,
    output reg         DONE
);

wire  [3:0]  cnt_x;
wire  [3:0]  cnt_y;
COUNTER_XY counter_inst (
    .CLK(CLK),
    .RST(RST),
    .cnt_x(cnt_x),
    .cnt_y(cnt_y),
    .CALC_DONE(CALC_DONE)
);

wire signed [15:0] x_normalized, y_normalized;

coord_processor u_x_proc (.coord_i(cnt_x), .out_o(x_normalized));
coord_processor u_y_proc (.coord_i(cnt_y), .out_o(y_normalized));

wire [15:0] eta; //1/RI
RI_LUT ri_lut_inst (
    .ri_i(RI),
    .eta_o(eta)
);

wire signed [15:0] y14, y8;
wire signed [15:0] x14, x8;

power_8_14_pipelined power_X (
    .clk(CLK),
    .rst(RST),
    .x_i(x_normalized), 
    .x14_o(x14),    
    .x8_o(x8) 
);

power_8_14_pipelined power_Y (
    .clk(CLK),
    .rst(RST),
    .x_i(y_normalized), 
    .x14_o(y14),    
    .x8_o(y8) 
);

// 建議定義：整數 4 bits + 小數 12 bits = 16 bits 基礎，但計算中擴展至 24 bits Q12.12
wire [23:0] gx2 = {8'd0, x14} << 2; // gx^2 = 4 * x^14
wire [23:0] gy2 = {8'd0, y14} << 2; // gy^2 = 4 * y^14

// g2 = gx^2 + gy^2 + 1
// 注意：1 在 Q4.12 格式下是 24'h001000
//So call A = g2_minus1 Q12.12
wire [23:0] g2_minus1 = gx2 + gy2 ; 
wire [23:0] g2 = g2_minus1 + 24'h001000;
// 保留 16 位小數 (Q8.16)，寬度 24 bits
wire [23:0] eta2 = (eta * eta) >>> 8;

//eta2A
reg [47:0] eta2A; //eta平方乘上A
reg [27:0] etaA; //eta乘上A

always @(posedge CLK or posedge RST) begin
    if(RST)
        eta2A <= 0;
        etaA <= 0;
    else
        eta2A <= (eta2 * g2_minus1) ; // eta * g2 後再右移 12 位元對齊小數部分 Q20.28
        etaA <= (eta * g2_minus1) >>> 12 ; // eta * g2 後再右移 12 位元對齊小數部分 Q15.12
end

wire [47:0] kgg_q28 = ({24'd0, g2} << 16) - eta2A; //Q20.28

DW_sqrt #(width,tc_mode) 
        (.a(),
         .root());



endmodule



module COUNTER_XY(
    input          CLK,
    input          RST,
    output reg  [3:0]  cnt_x,
    output reg  [3:0]  cnt_y,
    input          CALC_DONE
);

reg [3:0] cnt_x, cnt_y; // 4-bit 可以跑 0~15

// 當運算完成一條光線並寫入 SRAM 後
always @(posedge CLK or posedge RST) begin
    if (RST) begin
        cnt_x <= 4'd0;
        cnt_y <= 4'd0;
    end else if (CALC_DONE) begin
        if (cnt_x == 4'd15) begin
            cnt_x <= 4'd0;
            cnt_y <= cnt_y + 4'd1;
        end else begin
            cnt_x <= cnt_x + 4'd1;
        end
    end
end

endmodule

module RI_LUT (
    input  wire [3:0]  ri_i,   
    output reg  [15:0] eta_o   // 輸出 eta = 1/RI (Q4.12 格式) [cite: 147, 159]
);

    always @(*) begin
        case (ri_i)
            4'd2 : eta_o = 16'h0800;
            4'd3 : eta_o = 16'h0555;
            4'd4 : eta_o = 16'h0400;
            4'd5 : eta_o = 16'h0333;
            4'd6 : eta_o = 16'h02AB;
            4'd7 : eta_o = 16'h0249;
            4'd8 : eta_o = 16'h0200;
            4'd9 : eta_o = 16'h01C7;
            4'd10: eta_o = 16'h019A;
            4'd11: eta_o = 16'h0174;
            4'd12: eta_o = 16'h0155;
            4'd13: eta_o = 16'h013B;
            4'd14: eta_o = 16'h0125;
            4'd15: eta_o = 16'h0111;
            default: eta_o = 16'h0000;
        endcase
    end

endmodule

module coord_processor #(
    parameter INT_W = 4,    // 整數部分位元數 
    parameter FRAC_W = 12   // 小數部分位元數 
)(
    input  wire [3:0]           coord_i,    // 原始座標 X 或 Y (0~15) [cite: 68]
    output wire signed [INT_W+FRAC_W-1:0] out_o // 輸出 (X-8)/8 具符號定點數
);

    // 1. 將無號 4-bit 轉為具符號 5-bit，以安全處理 -8 
    wire signed [4:0] s_coord = $signed({1'b0, coord_i});
    
    // 2. 執行減法：範圍變成 -8 ~ 7
    wire signed [4:0] s_minus_8 = s_coord - 5'd8;

    // 3. 轉為 Q4.12 定點數格式
    // 將整數左移 12 位元 
    wire signed [INT_W+FRAC_W-1:0] q_format = {s_minus_8, {FRAC_W{1'b0}}};

    // 4. 算術右移 3 位元 (等於除以 8)
    // 使用 >>> 確保負數符號位元正確保留 
    assign out_o = q_format >>> 3;

endmodule

module signed_mul_trunc #(
    parameter INT_W = 4,
    parameter FRAC_W = 12,
    parameter KEEP_FRAC = 16 // 每一級運算後保留的小數位數
)(
    input  wire                         clk,
    input  wire signed [INT_W+FRAC_W-1:0] a_i,
    input  wire signed [INT_W+FRAC_W-1:0] b_i,
    output reg  signed [INT_W+KEEP_FRAC-1:0] out_o
);
    // 兩數相乘，位元數會相加
    // 例如 Q4.12 * Q4.12 = Q8.24
    localparam MUL_W = (INT_W + FRAC_W) * 2;
    wire signed [MUL_W-1:0] mul_res = a_i * b_i;

    always @(posedge clk) begin
        // 截斷邏輯：
        // 1. 捨棄過低精度的位元 (從原本的 FRAC_W*2 縮減回 KEEP_FRAC)
        // 2. 為了減少誤差，這裡建議加上 1/2 LSB 做四捨五入 (Rounding)
        out_o <= (mul_res + (1 << (FRAC_W*2 - KEEP_FRAC - 1))) >>> (FRAC_W*2 - KEEP_FRAC);
    end
endmodule

module power_8_14_pipelined #(
    parameter INT_W = 4,
    parameter FRAC_W = 12,
    parameter KEEP_FRAC = 16 
)(
    input  wire                         clk,
    input  wire                         rst,
    input  wire signed [INT_W+FRAC_W-1:0] x_i,     // 輸入 (X-8)/8
    output wire signed [INT_W+FRAC_W-1:0] x8_o,    // 輸出 (X-8)/8 ^ 8
    output wire signed [INT_W+FRAC_W-1:0] x14_o    // 輸出 (X-8)/8 ^ 14
);

    // --- 內部連線 ---
    wire signed [INT_W+KEEP_FRAC-1:0] x1_in;
    wire signed [INT_W+KEEP_FRAC-1:0] x2, x4, x6, x8, x14;
    
    // 輸入轉精度
    assign x1_in = (KEEP_FRAC > FRAC_W) ? (x_i <<< (KEEP_FRAC - FRAC_W)) : x_i;

    // --- 延遲對齊暫存器 ---
    reg signed [INT_W+KEEP_FRAC-1:0] x2_d1;
    reg signed [INT_W+KEEP_FRAC-1:0] x8_d1; // 將 Stage 3 算出的 x8 延遲一拍，與 Stage 4 的 x14 同步輸出

    always @(posedge clk) begin
        if (rst) begin
            x2_d1 <= 0;
            x8_d1 <= 0;
        end else begin
            x2_d1 <= x2;     // 用於 Stage 3 計算 x6
            x8_d1 <= x8;     // 用於與 x14 對齊輸出
        end
    end

    // --- 運算單元 ---

    // Stage 1: x^2 = x * x (Latency: 1)
    signed_mul_trunc #(.KEEP_FRAC(KEEP_FRAC)) u_mul_x2 (
        .clk(clk), .a_i(x1_in), .b_i(x1_in), .out_o(x2)
    );

    // Stage 2: x^4 = x^2 * x^2 (Latency: 2)
    signed_mul_trunc #(.KEEP_FRAC(KEEP_FRAC)) u_mul_x4 (
        .clk(clk), .a_i(x2), .b_i(x2), .out_o(x4)
    );

    // Stage 3: (Latency: 3)
    // x^8 = x^4 * x^4
    signed_mul_trunc #(.KEEP_FRAC(KEEP_FRAC)) u_mul_x8 (
        .clk(clk), .a_i(x4), .b_i(x4), .out_o(x8)
    );
    // x^6 = x^4 * x^2 (需用 x2_d1 對齊 x4)
    signed_mul_trunc #(.KEEP_FRAC(KEEP_FRAC)) u_mul_x6 (
        .clk(clk), .a_i(x4), .b_i(x2_d1), .out_o(x6)
    );

    // Stage 4: x^14 = x^8 * x^6 (Latency: 4)
    // 這裡直接使用 Stage 3 剛算出的 x8 與 x6
    signed_mul_trunc #(.KEEP_FRAC(KEEP_FRAC)) u_mul_x14 (
        .clk(clk), .a_i(x8), .b_i(x6), .out_o(x14)
    );

    // --- 輸出截斷與對齊 ---
    // x8_o 使用延遲過的 x8_d1，確保與 x14 同時在輸入 4 拍後出現
    assign x8_o  = x8_d1 >>> (KEEP_FRAC - FRAC_W);
    assign x14_o = x14    >>> (KEEP_FRAC - FRAC_W);

endmodule


module DW_div_pipe_inst(inst_clk, inst_rst_n, inst_en, inst_a, inst_b,
                        quotient_inst, remainder_inst, divide_by_0_inst );

  parameter inst_a_width = 21;
  parameter inst_b_width = 21;
  parameter inst_tc_mode = 0;
  parameter inst_rem_mode = 1;
  parameter inst_num_stages = 4;
  parameter inst_stall_mode = 0;
  parameter inst_rst_mode = 1;
  parameter inst_op_iso_mode = 0;

  input inst_clk;
  input inst_rst_n;
  input inst_en;
  input [inst_a_width-1 : 0] inst_a;
  input [inst_b_width-1 : 0] inst_b;
  output [inst_a_width-1 : 0] quotient_inst;
  output [inst_b_width-1 : 0] remainder_inst;
  output divide_by_0_inst;

  // Instance of DW_div_pipe
  DW_div_pipe #(inst_a_width,    inst_b_width,    inst_tc_mode,  inst_rem_mode,
                inst_num_stages, inst_stall_mode, inst_rst_mode, inst_op_iso_mode) 
    U1 (.clk(inst_clk), .rst_n(inst_rst_n), .en(inst_en),
        .a(inst_a), .b(inst_b), .quotient(quotient_inst),
        .remainder(remainder_inst), .divide_by_0(divide_by_0_inst) );
endmodule
