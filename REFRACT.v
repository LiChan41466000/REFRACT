


`include "/usr/cad/synopsys/synthesis/2025.06/dw/sim_ver/DW_sqrt.v"
module REFRACT(
    input  wire        CLK,
    input  wire        RST,
    input  wire [3:0]  RI,   
    output  [8:0]  SRAM_A,
    output   [15:0] SRAM_D,
    input  [15:0] SRAM_Q,   // unused
    output         SRAM_WE,
    output         DONE
);

wire  [3:0]  cnt_x;
wire  [3:0]  cnt_y;
wire CALC_DONE;
COUNTER_XY counter_inst (
    .CLK(CLK),
    .RST(RST),
    .cnt_x(cnt_x),
    .cnt_y(cnt_y),
    .CALC_DONE(CALC_DONE),
    .finish(DONE)
);

COUNTER_ONE_SHOT counter_one_shot_inst (
    .CLK(CLK),
    .RST(RST),
    .ONE_SHOT_OUT(CALC_DONE)
);

wire signed [15:0] x_normalized, y_normalized;

coord_processor u_x_proc (.coord_i(cnt_x), .out_o(x_normalized));
coord_processor u_y_proc (.coord_i(cnt_y), .out_o(y_normalized));

wire [3:0] x_d7, y_d8,x_d6,y_d6; //buf座標
COORD_DELAY coord_delay_inst (
    .clk(CLK),
    .rst(RST),
    .x_i(cnt_x),
    .y_i(cnt_y),
    .x_d7(x_d7),
    .y_d8(y_d8),
    .x_d6(x_d6),
    .y_d6(y_d6)
);



wire [15:0] eta; //1/RI
RI ri_inst (
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

reg signed [15:0] x_norm_d1, x_norm_d2, x_norm_d3, x_norm_d4;

always @(posedge CLK or posedge RST) begin
    if (RST) begin
        x_norm_d1 <= 16'd0;
        x_norm_d2 <= 16'd0;
        x_norm_d3 <= 16'd0;
        x_norm_d4 <= 16'd0;
    end else begin
        x_norm_d1 <= x_normalized; // 延遲 1 拍
        x_norm_d2 <= x_norm_d1;     // 延遲 2 拍
        x_norm_d3 <= x_norm_d2;     // 延遲 3 拍
        x_norm_d4 <= x_norm_d3;     // 延遲 4 拍
    end
end
reg signed [15:0] y_norm_d1, y_norm_d2, y_norm_d3, y_norm_d4;
always @(posedge CLK or posedge RST) begin
    if (RST) begin
        y_norm_d1 <= 16'd0;
        y_norm_d2 <= 16'd0;
        y_norm_d3 <= 16'd0;
        y_norm_d4 <= 16'd0;
    end else begin
        y_norm_d1 <= y_normalized; // 延遲 1 拍
        y_norm_d2 <= y_norm_d1;     // 延遲 2 拍
        y_norm_d3 <= y_norm_d2;     // 延遲 3 拍
        y_norm_d4 <= y_norm_d3;     // 延遲 4 拍
    end
end
// 此時 x_norm_d4 會與 power_X 輸出的 x8_o, x14_o 同步


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
reg [27:0] etaA_buf; //eta乘上A

always @(posedge CLK or posedge RST) begin
    if(RST) begin
        eta2A <= 0;
        etaA <= 0;
    end
    else begin
        eta2A <= (eta2 * g2_minus1) ; // eta * g2 後再右移 12 位元對齊小數部分 Q20.28
        etaA <= (eta * g2_minus1) >>> 12 ; // eta * g2 後再右移 12 位元對齊小數部分 Q15.12
        etaA_buf <= etaA; // 將 etaA 的值暫存一拍，提供給下一個時鐘週期使用
    end
end



wire [47:0] kgg = ({24'd0, g2} << 16) - eta2A; //Q20.28

localparam width = 48;
localparam tc_mode = 0;
reg  [47:0] kgg_reg; 
always @(posedge CLK or posedge RST) begin
     if(RST)
        kgg_reg <= 48'd0;
     else
        kgg_reg <= kgg;
end
wire [23:0] sqrt_kgg;//Q10.14

DW_sqrt #(width,tc_mode) 
    DW_sqrt_inst   (.a(kgg_reg),
                    .root(sqrt_kgg));

//Z
reg [16:0] z; //Q5.12
always @(posedge CLK or posedge RST) begin
    if(RST)
        z <= 17'd0;
    else
        z <= 17'd6 - ({1'd0, x8} << 1) - ({1'd0, y8} << 1) ; 
end


//x7 y7 
reg signed [15:0] y7;
reg signed [15:0] x7;
always @(posedge CLK or posedge RST) begin
    if(RST) begin
        x7 <= 16'd0;
        y7 <= 16'd0;
    end else begin
        x7 <= ({12'd0,x8} << 12) / x_norm_d4; // Q5.12 * Q4.12 = Q9.12
        y7 <= ({12'd0,y8} << 12) / y_norm_d4; // Q5.12 * Q4.12 = Q9.12
    end
end

//Z乘上gx gy
reg [31:0] z_gx; //Q5.12 * Q4.12 = Q9.24
reg [31:0] z_gy; //Q5.12 * Q4.12 = Q9.24
always @(posedge CLK or posedge RST) begin
    if(RST) begin
        z_gx <= 0;
        z_gy <= 0;
    end else begin
        z_gx <= z * x7; // Q5.12 * Q4.12 = Q9.24
        z_gy <= z * y7; // Q5.12 * Q4.12 = Q9.24
    end
end

//M
reg [26:0] M;
always @(posedge CLK or posedge RST) begin
    if(RST) begin
        M <= 0;
    end
    else begin
        M <= (eta-sqrt_kgg) / (etaA_buf + sqrt_kgg); //Q15.12 / Q15.12 = Q15.12
    end
end



// ZX ZY
reg [72:0] zx,zy; //Q28.48
always @(posedge CLK or posedge RST) begin
    if(RST) begin
        zx <= 0;
        zy <= 0;
    end
    else begin
        zx <= ( {24'd0,x_d6}<<24 + M * z_gx); // 4.36+ Q15.12 * Q9.24 = Q25.48
        zy <= ( {24'd0,y_d6}<<24 + M * z_gy); 
    end
end

wire [15:0] zx_out = zx[39:24]; // 將 zx 從 Q28.48 對齊回 Q4.12
wire [15:0] zy_out = zy[39:24]; // 將 zy 從 Q28.48 對齊回 Q4.12
reg  [15:0] zy_out_buf; // 將 zy 從 Q28.48 對齊回 Q4.12

always @(posedge CLK or posedge RST) begin
        if(RST) begin
        zy_out_buf <= 0;
    end
    else begin
        zy_out_buf <= zy_out;
    end
end

assign SRAM_D = (CALC_DONE)?zx_out:zy_out_buf;

assign SRAM_WE = 1'd1; // 持續寫入，SRAM 模組內部會根據地址決定是否真正寫入

assign SRAM_A = (x_d7+ y_d8*16)*2 + CALC_DONE ; // 寫入 zx 時使用 x_d7, y_d8；寫入 zy 時使用 x_d6, y_d6

endmodule

module COORD_DELAY (
    input  wire       clk,
    input  wire       rst,
    input  wire [3:0] x_i,     // 來自 COUNTER_XY 的 cnt_x [cite: 68]
    input  wire [3:0] y_i,     // 來自 COUNTER_XY 的 cnt_y [cite: 68]
    output wire [3:0] x_d7,  // 延遲 10 拍後的 X
    output wire [3:0] y_d8,  // 延遲 10 拍後的 Y
    output wire [3:0] x_d6,
    output wire [3:0] y_d6
);

    // 建立 10 階的陣列暫存器
    reg [3:0] x_pipe [0:8];
    reg [3:0] y_pipe [0:8];
    integer i;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 8; i = i + 1) begin
                x_pipe[i] <= 4'd0;
                y_pipe[i] <= 4'd0;
            end
        end else begin
            // 第一階接輸入
            x_pipe[0] <= x_i;
            y_pipe[0] <= y_i;
            
            // 後續每一階接前一階 (Shift Logic)
            for (i = 1; i < 8; i = i + 1) begin
                x_pipe[i] <= x_pipe[i-1];
                y_pipe[i] <= y_pipe[i-1];
            end
        end
    end

    // 輸出最後一階
    assign x_d6 = x_pipe[6];
    assign y_d6 = y_pipe[6];
    assign x_d7 = x_pipe[7];
    assign y_d8 = y_pipe[8];

endmodule

module COUNTER_XY(
    input          CLK,
    input          RST,
    output reg  [3:0]  cnt_x,
    output reg  [3:0]  cnt_y,
    input          CALC_DONE,
    output reg         finish
);

reg [3:0] cnt_x, cnt_y; // 4-bit 可以跑 0~15

// 當運算完成一條光線並寫入 SRAM 後
always @(posedge CLK or posedge RST) begin
    if (RST) begin
        cnt_x <= 4'd0;
        cnt_y <= 4'd0;
        finish <= 1'b0;
    end else if (~CALC_DONE) begin
        if (cnt_x == 4'd15) begin
            if (cnt_y == 4'd15) begin
                finish <= 1'b1; // 全部計算完成
            end
            else begin
            cnt_x <= 4'd0;
            cnt_y <= cnt_y + 4'd1;
            end
        end 
        else begin
            cnt_x <= cnt_x + 4'd1;
        end
    end
end

endmodule

module COUNTER_ONE_SHOT(
    input          CLK,
    input          RST,
    output reg     ONE_SHOT_OUT
);

 always @(posedge CLK or posedge RST) begin
    if(RST) begin
        ONE_SHOT_OUT <= 1'b0;
    end else begin
        ONE_SHOT_OUT <= ONE_SHOT_OUT+1'b1; // 在第一個時鐘週期後持續輸出高電位
    end
 end


endmodule


module RI (
    input  wire [7:0]  ri_i,   // 除數
    output wire [15:0] eta_o   // 輸出 eta = (1<<12) / ri_i
);

    // 定義中間變數，手動展開除法步驟 (12階，因為是 Q4.12)
    reg [19:0] remainder;
    reg [15:0] quotient;
    integer i;

    always @(*) begin
        // 初始化：被除數是 1.0 (Q4.12 格式即 2^12 = 4096)
        // 我們從高位開始遞推
        remainder = 20'd4096; 
        quotient = 16'd0;

        if (ri_i == 8'd0) begin
            quotient = 16'hFFFF; // 處理除以 0
        end else begin
            // 組合邏輯迴圈：這在合成時會被展開成多級加法器/減法器
            for (i = 15; i >= 0; i = i - 1) begin
                if (remainder >= (ri_i << i)) begin
                    remainder = remainder - (ri_i << i);
                    quotient[i] = 1'b1;
                end else begin
                    quotient[i] = 1'b0;
                end
            end
        end
    end

    assign eta_o = quotient;

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

