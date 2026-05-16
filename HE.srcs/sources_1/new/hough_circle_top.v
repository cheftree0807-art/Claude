// ============================================================================
// hough_circle_top - 霍夫变换圆检测 FPGA 模块
// Xilinx Zynq-7000, Vivado 2018.3
// ============================================================================
// 接口: AXI4-Stream, 8位灰度像素
// 两遍遍历: Pass1=边缘检测+霍夫投票+峰值检测, Pass2=叠加标记
// ============================================================================

`timescale 1ns / 1ps

module hough_circle_top #(
    parameter IMG_WIDTH       = 1280,
    parameter IMG_HEIGHT      = 720,
    parameter DS_SHIFT        = 3,           // 8倍降采样
    parameter ACCUM_CX        = 160,         // 1280/8
    parameter ACCUM_CY        = 90,          // 720/8
    parameter R_MIN           = 10,
    parameter R_STEP          = 10,
    parameter NUM_RADII       = 10,          // 10,20,...,100
    parameter EDGE_THRESHOLD  = 80,
    parameter PEAK_THRESHOLD  = 40,
    parameter MAX_CIRCLES     = 256
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         s_axis_tvalid,
    output reg          s_axis_tready,
    input  wire  [7:0]  s_axis_tdata,
    input  wire         s_axis_tlast,
    output reg          m_axis_tvalid,
    input  wire         m_axis_tready,
    output reg  [7:0]   m_axis_tdata,
    output reg          m_axis_tlast
);

    // =========================================================================
    // 状态机
    // =========================================================================
    localparam [2:0]
        S_IDLE            = 3'd0,
        S_PASS1_SOBEL     = 3'd1,
        S_VOTE_START      = 3'd2,
        S_VOTE_LOOP       = 3'd3,
        S_PASS1_DONE      = 3'd4,
        S_PEAK_SCAN       = 3'd5,
        S_CLEAR_BITMAP    = 3'd6,
        S_RASTERIZE       = 3'd7,
        S_RASTERIZE_WAIT  = 3'd8,
        S_WAIT_PASS2      = 3'd9,
        S_PASS2_OVERLAY   = 3'd10,
        S_DONE            = 3'd11;

    reg [3:0] state;
    reg [3:0] return_state;  // 子流程返回

    // =========================================================================
    // 行缓冲区 (2 × BRAM36 SDP)
    // =========================================================================
    (* ram_style = "block" *) reg [7:0] line_buf0 [0:IMG_WIDTH-1];
    (* ram_style = "block" *) reg [7:0] line_buf1 [0:IMG_WIDTH-1];
    reg [7:0]  lb0_rd_data, lb1_rd_data;
    reg [10:0] lb_rd_addr;
    reg [10:0] lb_wr_addr;
    reg        lb_wr_en;
    reg [7:0]  lb0_wr_data, lb1_wr_data;

    // =========================================================================
    // 3×3 窗口
    // =========================================================================
    reg [7:0] win [0:2][0:2];
    reg       win_valid;

    // =========================================================================
    // Sobel 结果 (流水线)
    // =========================================================================
    reg signed [11:0] gx_r, gy_r;
    reg [10:0] grad_mag_r;
    reg        is_edge_r;
    reg [2:0]  grad_dir_r;    // 8方向
    reg        sobel_valid_r;

    // 降采样坐标
    reg [6:0]  x_ds, y_ds;    // 160×90 空间
    reg        x_ds_valid;

    // =========================================================================
    // 霍夫累加器: 10平面 × 16384×9位 (40 BRAM36)
    // =========================================================================
    localparam ACCUM_DEPTH = 16384;  // 4K×4=16K

    // 平面展开为独立数组以使 BRAM 正确推断
    (* ram_style = "block" *) reg [8:0] accum_0 [0:ACCUM_DEPTH-1];
    (* ram_style = "block" *) reg [8:0] accum_1 [0:ACCUM_DEPTH-1];
    (* ram_style = "block" *) reg [8:0] accum_2 [0:ACCUM_DEPTH-1];
    (* ram_style = "block" *) reg [8:0] accum_3 [0:ACCUM_DEPTH-1];
    (* ram_style = "block" *) reg [8:0] accum_4 [0:ACCUM_DEPTH-1];
    (* ram_style = "block" *) reg [8:0] accum_5 [0:ACCUM_DEPTH-1];
    (* ram_style = "block" *) reg [8:0] accum_6 [0:ACCUM_DEPTH-1];
    (* ram_style = "block" *) reg [8:0] accum_7 [0:ACCUM_DEPTH-1];
    (* ram_style = "block" *) reg [8:0] accum_8 [0:ACCUM_DEPTH-1];
    (* ram_style = "block" *) reg [8:0] accum_9 [0:ACCUM_DEPTH-1];

    // 累加器读写控制
    reg [3:0]  ac_r_idx;           // 0..9 选择平面
    reg [13:0] ac_addr;            // 平面内地址 (cy*160 + cx)
    reg [8:0]  ac_rd_data;         // 读出数据
    reg        ac_wr_en;
    reg [8:0]  ac_wr_data;

    // 投票流水线
    reg [3:0]  vt_r_idx;           // 当前投票半径索引
    reg signed [5:0] vt_cx_offs;   // 圆心X偏移 (有符号)
    reg signed [5:0] vt_cy_offs;   // 圆心Y偏移 (有符号)
    reg [6:0]  vt_cx_ds;           // 候选圆心X (降采样坐标)
    reg [6:0]  vt_cy_ds;           // 候选圆心Y (降采样坐标)
    reg [2:0]  vt_state;           // 投票流水线级数

    // =========================================================================
    // 圆列表 (256×28位 BRAM)
    // =========================================================================
    (* ram_style = "block" *) reg [27:0] circle_ram [0:MAX_CIRCLES-1];
    // circle_ram[*] = { cx[10:0], cy[9:0], r[6:0] }  共28位
    reg [7:0]  cr_count;
    reg [7:0]  cr_wr_ptr, cr_rd_ptr;

    // =========================================================================
    // 叠加位图: 1280×720×1位 = 921600位 = 102400×9位 (25 BRAM36)
    // =========================================================================
    localparam OV_DEPTH = 102400;  // ceil(1280*720/9)
    (* ram_style = "block" *) reg [8:0] overlay_bm [0:OV_DEPTH-1];
    reg [16:0] ov_clear_cnt;

    // =========================================================================
    // 位置计数器
    // =========================================================================
    reg [10:0] pos_x;       // 0..1279
    reg [9:0]  pos_y;       // 0..719
    reg [20:0] pos_pixel;   // 0..921599
    reg        frame_start;

    // =========================================================================
    // 峰值检测
    // =========================================================================
    reg [3:0]  pk_r_idx;
    reg [6:0]  pk_cx, pk_cy;
    reg [13:0] pk_addr;
    reg [8:0]  pk_center_val;
    reg [8:0]  pk_neighbors [0:7];
    reg        pk_has_data;
    reg [3:0]  pk_scan_state;

    // =========================================================================
    // 画圆状态机 (Bresenham)
    // =========================================================================
    reg [10:0] br_cx;          // 圆心X (全分辨率)
    reg [9:0]  br_cy;          // 圆心Y (全分辨率)
    reg [6:0]  br_r;           // 半径
    reg [6:0]  br_rr;          // 当前半径 (画3层)
    reg [10:0] br_x, br_y;     // Bresenham 坐标
    reg [11:0] br_d;           // 决策参数
    reg [2:0]  br_layer;       // 0..2 (画3层)
    reg [2:0]  br_oct;         // 八分圆 0..7
    reg [3:0]  br_state;

    // 画圆像素坐标
    reg signed [11:0] br_px [0:7];   // 8个对称点X
    reg signed [10:0] br_py [0:7];   // 8个对称点Y
    reg [2:0]  br_pt_idx;            // 当前处理的对称点
    reg [20:0] br_pixel_idx;         // 像素索引
    reg [16:0] br_word_addr;         // 叠加位图字地址
    reg [3:0]  br_bit_pos;           // 叠加位图位位置

    reg [8:0]  br_ov_rd_data;   // 叠加位图读出数据
    // =========================================================================
    reg [20:0] p2_pixel_idx;
    reg [16:0] p2_word_addr;
    reg [3:0]  p2_bit_pos;
    reg [8:0]  p2_ov_word;
    reg        p2_valid;
    reg [7:0]  p2_orig_data;
    reg        p2_orig_valid;
    reg        p2_orig_last;

    // =========================================================================
    // 辅助信号
    // =========================================================================
    reg [31:0] cycle_cnt;     // 调试用周期计数

    // 查找表: 方向 → (cos_sign, sin_sign, use_cos_as_major)
    // 8方向梯度 → 圆心偏移方向 (指向圆心在梯度反方向)
    // dir 0: 0°    (向右)    → 圆心在左侧  (-r, 0)
    // dir 1: 45°   (右下)    → 圆心在左上  (-r, -r)
    // dir 2: 90°   (向下)    → 圆心在上方  (0, -r)
    // dir 3: 135°  (左下)    → 圆心在右上  (+r, -r)
    // dir 4: 180°  (向左)    → 圆心在右侧  (+r, 0)
    // dir 5: 225°  (左上)    → 圆心在右下  (+r, +r)
    // dir 6: 270°  (向上)    → 圆心在下方  (0, +r)
    // dir 7: 315°  (右上)    → 圆心在左下  (-r, +r)

    function [5:0] get_cx_offs;
        input [2:0] dir;
        input [3:0] r_idx;
        reg [6:0] r_val;
        begin
            r_val = (r_idx + 4'd1) * 7'd10;  // r = (r_idx+1)*10
            case (dir)
                3'd0: get_cx_offs = -{1'b0, r_val[5:0]};   // -r
                3'd1: get_cx_offs = -{1'b0, r_val[5:0]};   // -r
                3'd2: get_cx_offs = 6'sd0;
                3'd3: get_cx_offs =  {1'b0, r_val[5:0]};   // +r
                3'd4: get_cx_offs =  {1'b0, r_val[5:0]};   // +r
                3'd5: get_cx_offs =  {1'b0, r_val[5:0]};   // +r
                3'd6: get_cx_offs = 6'sd0;
                3'd7: get_cx_offs = -{1'b0, r_val[5:0]};   // -r
                default: get_cx_offs = 6'sd0;
            endcase
        end
    endfunction

    function [5:0] get_cy_offs;
        input [2:0] dir;
        input [3:0] r_idx;
        reg [6:0] r_val;
        begin
            r_val = (r_idx + 4'd1) * 7'd10;
            case (dir)
                3'd0: get_cy_offs = 6'sd0;
                3'd1: get_cy_offs = -{1'b0, r_val[5:0]};   // -r
                3'd2: get_cy_offs = -{1'b0, r_val[5:0]};   // -r
                3'd3: get_cy_offs = -{1'b0, r_val[5:0]};   // -r
                3'd4: get_cy_offs = 6'sd0;
                3'd5: get_cy_offs =  {1'b0, r_val[5:0]};   // +r
                3'd6: get_cy_offs =  {1'b0, r_val[5:0]};   // +r
                3'd7: get_cy_offs =  {1'b0, r_val[5:0]};   // +r
                default: get_cy_offs = 6'sd0;
            endcase
        end
    endfunction

    // =========================================================================
    // Sobel 梯度计算 (组合逻辑)
    // =========================================================================
    wire [10:0] sobel_gx_pos = {3'd0, win[0][2]} + {2'd0, win[1][2], 1'b0} + {3'd0, win[2][2]};
    wire [10:0] sobel_gx_neg = {3'd0, win[0][0]} + {2'd0, win[1][0], 1'b0} + {3'd0, win[2][0]};
    wire signed [11:0] sobel_gx = {1'b0, sobel_gx_pos} - {1'b0, sobel_gx_neg};

    wire [10:0] sobel_gy_pos = {3'd0, win[2][0]} + {2'd0, win[2][1], 1'b0} + {3'd0, win[2][2]};
    wire [10:0] sobel_gy_neg = {3'd0, win[0][0]} + {2'd0, win[0][1], 1'b0} + {3'd0, win[0][2]};
    wire signed [11:0] sobel_gy = {1'b0, sobel_gy_pos} - {1'b0, sobel_gy_neg};

    wire [10:0] sobel_abs_gx = sobel_gx[11] ? (~sobel_gx[10:0] + 1'b1) : sobel_gx[10:0];
    wire [10:0] sobel_abs_gy = sobel_gy[11] ? (~sobel_gy[10:0] + 1'b1) : sobel_gy[10:0];
    wire [10:0] sobel_mag   = sobel_abs_gx + sobel_abs_gy;
    wire        sobel_is_edge = sobel_mag > EDGE_THRESHOLD;

    // 方向量化: 基于 |Gy|/|Gx| 比较和符号
    wire [2:0] sobel_dir;
    assign sobel_dir[2] = sobel_gy[11];
    assign sobel_dir[1] = sobel_gx[11];
    assign sobel_dir[0] = (sobel_abs_gy > sobel_abs_gx);

    // =========================================================================
    // 主状态机
    // =========================================================================
    integer i, j, k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            return_state   <= S_IDLE;
            s_axis_tready  <= 1'b0;
            m_axis_tvalid  <= 1'b0;
            m_axis_tdata   <= 8'd0;
            m_axis_tlast   <= 1'b0;

            pos_x      <= 11'd0;
            pos_y      <= 10'd0;
            pos_pixel  <= 21'd0;
            frame_start<= 1'b0;

            lb_rd_addr <= 11'd0;
            lb_wr_addr <= 11'd0;
            lb_wr_en   <= 1'b0;
            lb0_wr_data<= 8'd0;
            lb1_wr_data<= 8'd0;
            lb0_rd_data<= 8'd0;
            lb1_rd_data<= 8'd0;

            for (i = 0; i < 3; i = i + 1)
                for (j = 0; j < 3; j = j + 1)
                    win[i][j] <= 8'd0;
            win_valid    <= 1'b0;

            gx_r         <= 12'sd0;
            gy_r         <= 12'sd0;
            grad_mag_r   <= 11'd0;
            is_edge_r    <= 1'b0;
            grad_dir_r   <= 3'd0;
            sobel_valid_r<= 1'b0;
            x_ds         <= 7'd0;
            y_ds         <= 7'd0;
            x_ds_valid   <= 1'b0;

            ac_r_idx     <= 4'd0;
            ac_addr      <= 14'd0;
            ac_wr_en     <= 1'b0;
            ac_wr_data   <= 9'd0;
            vt_r_idx     <= 4'd0;
            vt_cx_offs   <= 6'sd0;
            vt_cy_offs   <= 6'sd0;
            vt_cx_ds     <= 7'd0;
            vt_cy_ds     <= 7'd0;
            vt_state     <= 3'd0;

            cr_count     <= 8'd0;
            cr_wr_ptr    <= 8'd0;
            cr_rd_ptr    <= 8'd0;

            ov_clear_cnt <= 17'd0;

            pk_r_idx     <= 4'd0;
            pk_cx        <= 7'd0;
            pk_cy        <= 7'd0;
            pk_scan_state<= 4'd0;

            br_state     <= 4'd0;
            br_ov_rd_data<= 9'd0;
            br_layer     <= 3'd0;
            br_oct       <= 3'd0;
            br_pt_idx    <= 3'd0;
            br_busy      <= 1'b0;

            p2_pixel_idx <= 21'd0;
            p2_valid     <= 1'b0;
            p2_orig_valid<= 1'b0;
            p2_orig_last <= 1'b0;

            cycle_cnt    <= 32'd0;

            // 清零累加器 (上电初始化)
            // 注意: 实际BRAM初始化需要在上电时用 reset 信号触发
            // Vivado BRAM 支持 initial block 预初始化
        end else begin
            cycle_cnt <= cycle_cnt + 32'd1;

            // 默认输出
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;

            // =============================================================
            // 行缓冲区持续更新 (S_PASS1_SOBEL 状态)
            // =============================================================
            if (state == S_PASS1_SOBEL || state == S_VOTE_START || state == S_VOTE_LOOP) begin
                // 读行缓冲 (读旧值到窗口)
                lb0_rd_data <= line_buf0[lb_rd_addr];
                lb1_rd_data <= line_buf1[lb_rd_addr];

                // 写行缓冲 (移位: 新像素→lb0, lb0旧值→lb1)
                line_buf0[lb_wr_addr] <= lb0_wr_data;
                line_buf1[lb_wr_addr] <= lb1_wr_data;
            end

            // =============================================================
            // 主状态转移
            // =============================================================
            case (state)
                // ---------------------------------------------------------
                // S_IDLE: 等待第一帧
                // ---------------------------------------------------------
                S_IDLE: begin
                    s_axis_tready <= 1'b1;
                    pos_x     <= 11'd0;
                    pos_y     <= 10'd0;
                    pos_pixel <= 21'd0;
                    cr_count  <= 8'd0;
                    cr_wr_ptr <= 8'd0;
                    cr_rd_ptr <= 8'd0;

                    if (s_axis_tvalid) begin
                        state      <= S_PASS1_SOBEL;
                        frame_start <= 1'b1;
                    end
                end

                // ---------------------------------------------------------
                // S_PASS1_SOBEL: 第一遍 — Sobel边缘检测
                // ---------------------------------------------------------
                S_PASS1_SOBEL: begin
                    s_axis_tready <= 1'b1;
                    frame_start   <= 1'b0;

                    lb_rd_addr <= pos_x;
                    lb_wr_addr <= pos_x;
                    lb0_wr_data <= s_axis_tdata;
                    lb1_wr_data <= lb0_rd_data;

                    // 3×3窗口移位 (行流水)
                    win[0][0] <= win[0][1];
                    win[0][1] <= win[0][2];
                    win[0][2] <= lb0_rd_data;
                    win[1][0] <= win[1][1];
                    win[1][1] <= win[1][2];
                    win[1][2] <= line_buf0[lb_rd_addr];
                    win[2][0] <= win[2][1];
                    win[2][1] <= win[2][2];
                    win[2][2] <= lb1_rd_data;

                    // Sobel 流水线寄存器
                    gx_r         <= sobel_gx;
                    gy_r         <= sobel_gy;
                    grad_mag_r   <= sobel_mag;
                    is_edge_r    <= sobel_is_edge;
                    grad_dir_r   <= sobel_dir;
                    sobel_valid_r <= (pos_y >= 10'd2 && pos_x >= 11'd2 &&
                                      pos_x < IMG_WIDTH - 1 && pos_y < IMG_HEIGHT);

                    // 降采样坐标
                    x_ds <= pos_x[9:3];
                    y_ds <= pos_y[8:3];
                    x_ds_valid <= sobel_valid_r;

                    // 处理边缘像素: 启动投票
                    if (is_edge_r && sobel_valid_r) begin
                        state     <= S_VOTE_START;
                        vt_r_idx  <= 4'd0;
                        vt_state  <= 3'd0;
                        vt_cx_offs <= get_cx_offs(grad_dir_r, 4'd0);
                        vt_cy_offs <= get_cy_offs(grad_dir_r, 4'd0);
                        return_state <= S_PASS1_SOBEL;
                    end

                    // 帧结束
                    if (s_axis_tlast) begin
                        state <= S_PASS1_DONE;
                        pk_r_idx <= 4'd0;
                        pk_cx    <= 7'd0;
                        pk_cy    <= 7'd0;
                    end

                    // 像素计数
                    if (s_axis_tvalid) begin
                        if (pos_x == IMG_WIDTH - 1) begin
                            pos_x <= 11'd0;
                            pos_y <= pos_y + 10'd1;
                        end else begin
                            pos_x <= pos_x + 11'd1;
                        end
                        pos_pixel <= pos_pixel + 21'd1;
                    end
                end

                // ---------------------------------------------------------
                // S_VOTE_START: 投票初始化 (读取累加器)
                // ---------------------------------------------------------
                S_VOTE_START: begin
                    s_axis_tready <= 1'b0;  // 反压输入 (投票期间暂停数据流)
                    // 计算下采样候选圆心
                    // cx_ds = x_ds + cx_offs (cx_offs 已经是 r*cos(θ)/8)
                    // 注意: cx_offs/cy_offs 需要除以8来匹配降采样坐标
                    // 实际使用: r_val ≈ (r_idx+1)*10 / 8 = (r_idx+1)*1.25
                    vt_cx_ds <= x_ds + {{1{vt_cx_offs[5]}}, vt_cx_offs[5:3]};
                    vt_cy_ds <= y_ds + {{1{vt_cy_offs[5]}}, vt_cy_offs[5:3]};
                    // 钳位
                    if (x_ds + {{1{vt_cx_offs[5]}}, vt_cx_offs[5:3]} >= ACCUM_CX)
                        vt_cx_ds <= ACCUM_CX - 1;
                    if (y_ds + {{1{vt_cy_offs[5]}}, vt_cy_offs[5:3]} >= ACCUM_CY)
                        vt_cy_ds <= ACCUM_CY - 1;

                    // 计算累加器地址
                    ac_r_idx <= vt_r_idx;
                    // ac_addr = cy_ds * 160 + cx_ds = cy_ds << 7 + cy_ds << 5 + cx_ds
                    ac_addr  <= (vt_cy_ds << 7) + (vt_cy_ds << 5) + vt_cx_ds;

                    vt_state <= 3'd1;
                    state    <= S_VOTE_LOOP;
                end

                // ---------------------------------------------------------
                // S_VOTE_LOOP: 投票循环 (读-增量-写)
                // ---------------------------------------------------------
                S_VOTE_LOOP: begin
                    s_axis_tready <= 1'b0;

                    case (vt_state)
                        3'd1: begin  // 等待BRAM读出
                            vt_state <= 3'd2;
                        end
                        3'd2: begin  // 增量
                            // 从对应累加器平面读取数据并+1
                            if (ac_rd_data < 9'd511)
                                ac_wr_data <= ac_rd_data + 9'd1;
                            else
                                ac_wr_data <= 9'd511;  // 饱和
                            ac_wr_en   <= 1'b1;
                            vt_state   <= 3'd3;
                        end
                        3'd3: begin  // 写回完成
                            ac_wr_en <= 1'b0;
                            // 下一个半径
                            if (vt_r_idx == NUM_RADII - 1) begin
                                // 所有半径投票完成
                                state <= return_state;
                                s_axis_tready <= 1'b1;
                            end else begin
                                vt_r_idx <= vt_r_idx + 4'd1;
                                vt_state <= 3'd1;
                                // 更新圆心偏移
                                state <= S_VOTE_START;
                            end
                        end
                        default: vt_state <= 3'd0;
                    endcase
                end

                // ---------------------------------------------------------
                // S_PASS1_DONE: 第一遍完成, 进入峰值检测
                // ---------------------------------------------------------
                S_PASS1_DONE: begin
                    state <= S_PEAK_SCAN;
                    pk_r_idx <= 4'd0;
                    pk_cx    <= 7'd0;
                    pk_cy    <= 7'd0;
                    ac_r_idx <= 4'd0;
                    ac_addr  <= 14'd0;
                    pk_scan_state <= 4'd0;
                end

                // ---------------------------------------------------------
                // S_PEAK_SCAN: 扫描累加器检测峰值
                // ---------------------------------------------------------
                S_PEAK_SCAN: begin
                    case (pk_scan_state)
                        4'd0: begin  // 准备读
                            ac_r_idx <= pk_r_idx;
                            ac_addr  <= (pk_cy * 8'd160) + pk_cx;
                            pk_scan_state <= 4'd1;
                        end
                        4'd1: begin  // 等待读出
                            pk_scan_state <= 4'd2;
                        end
                        4'd2: begin  // 检查阈值
                            if (ac_rd_data > PEAK_THRESHOLD && cr_count < MAX_CIRCLES) begin
                                // 记录圆参数
                                // cx = cx_ds * 8 + 4 (全分辨率), cy同理
                                circle_ram[cr_wr_ptr] <= {
                                    pk_cx * 11'd8 + 11'd4,   // cx [27:17]
                                    pk_cy * 10'd8 + 10'd4,   // cy [16:7]
                                    (pk_r_idx * 7'd10 + 7'd10) // r [6:0]
                                };
                                cr_wr_ptr <= cr_wr_ptr + 8'd1;
                                cr_count  <= cr_count + 8'd1;
                            end
                            // 下一个单元
                            pk_scan_state <= 4'd3;
                        end
                        4'd3: begin
                            if (pk_cx < ACCUM_CX - 1) begin
                                pk_cx <= pk_cx + 7'd1;
                            end else begin
                                pk_cx <= 7'd0;
                                if (pk_cy < ACCUM_CY - 1) begin
                                    pk_cy <= pk_cy + 7'd1;
                                end else begin
                                    pk_cy <= 7'd0;
                                    if (pk_r_idx < NUM_RADII - 1) begin
                                        pk_r_idx <= pk_r_idx + 4'd1;
                                    end else begin
                                        // 扫描完成 → 清零位图
                                        state <= S_CLEAR_BITMAP;
                                        ov_clear_cnt <= 17'd0;
                                    end
                                end
                            end
                            pk_scan_state <= 4'd0;
                        end
                        default: pk_scan_state <= 4'd0;
                    endcase
                end

                // ---------------------------------------------------------
                // S_CLEAR_BITMAP: 清零叠加位图
                // ---------------------------------------------------------
                S_CLEAR_BITMAP: begin
                    overlay_bm[ov_clear_cnt] <= 9'd0;
                    if (ov_clear_cnt == OV_DEPTH - 1) begin
                        state <= S_RASTERIZE;
                        cr_rd_ptr <= 8'd0;
                        br_state  <= 4'd0;
                    end else begin
                        ov_clear_cnt <= ov_clear_cnt + 17'd1;
                    end
                end

                // ---------------------------------------------------------
                // S_RASTERIZE: 画圆到叠加位图 (Bresenham 中点算法)
                // ---------------------------------------------------------
                S_RASTERIZE: begin
                    case (br_state)
                        4'd0: begin  // 取下一个圆
                            if (cr_rd_ptr < cr_count) begin
                                br_cx   <= circle_ram[cr_rd_ptr][27:17];
                                br_cy   <= circle_ram[cr_rd_ptr][16:7];
                                br_r    <= circle_ram[cr_rd_ptr][6:0];
                                br_rr   <= circle_ram[cr_rd_ptr][6:0] - 7'd1;
                                br_layer <= 3'd0;
                                br_state <= 4'd1;
                                cr_rd_ptr <= cr_rd_ptr + 8'd1;
                            end else begin
                                // 画完 → 等第二遍
                                state <= S_WAIT_PASS2;
                                s_axis_tready <= 1'b1;
                            end
                        end
                        4'd1: begin  // 初始化 Bresenham
                            br_x <= 11'd0;
                            br_y <= {4'd0, br_rr};
                            br_d <= 12'd1 - {5'd0, br_rr};
                            br_oct <= 3'd0;
                            br_pt_idx <= 3'd0;
                            br_state <= 4'd2;
                        end
                        4'd2: begin  // 生成8个对称点
                            // 计算对称点坐标
                            // (cx+x, cy+y), (cx-x, cy+y), (cx+x, cy-y), (cx-x, cy-y)
                            // (cx+y, cy+x), (cx-y, cy+x), (cx+y, cy-x), (cx-y, cy-x)
                            case (br_oct)
                                3'd0: begin br_px[0] <= br_cx + br_x; br_py[0] <= br_cy + br_y; end
                                3'd1: begin br_px[0] <= br_cx - br_x; br_py[0] <= br_cy + br_y; end
                                3'd2: begin br_px[0] <= br_cx + br_x; br_py[0] <= br_cy - br_y; end
                                3'd3: begin br_px[0] <= br_cx - br_x; br_py[0] <= br_cy - br_y; end
                                3'd4: begin br_px[0] <= br_cx + br_y; br_py[0] <= br_cy + br_x; end
                                3'd5: begin br_px[0] <= br_cx - br_y; br_py[0] <= br_cy + br_x; end
                                3'd6: begin br_px[0] <= br_cx + br_y; br_py[0] <= br_cy - br_x; end
                                3'd7: begin br_px[0] <= br_cx - br_y; br_py[0] <= br_cy - br_x; end
                            endcase
                            br_state <= 4'd3;
                        end
                        4'd3: begin  // 读叠加位图
                            if (br_px[0] >= 0 && br_px[0] < IMG_WIDTH &&
                                br_py[0] >= 0 && br_py[0] < IMG_HEIGHT) begin
                                br_pixel_idx <= br_py[0] * 1280 + br_px[0];
                                br_word_addr <= (br_py[0] * 1280 + br_px[0]) / 17'd9;
                                br_bit_pos   <= (br_py[0] * 1280 + br_px[0]) % 4'd9;
                                // BRAM读延迟1周期
                                br_ov_rd_data <= overlay_bm[(br_py[0] * 1280 + br_px[0]) / 17'd9];
                            end
                            br_state <= 4'd4;
                        end
                        4'd4: begin  // 写叠加位图 (读-改-写)
                            if (br_px[0] >= 0 && br_px[0] < IMG_WIDTH &&
                                br_py[0] >= 0 && br_py[0] < IMG_HEIGHT) begin
                                overlay_bm[br_word_addr] <= br_ov_rd_data | (9'd1 << br_bit_pos);
                            end
                            // 下一个八分圆
                            if (br_oct == 3'd7) begin
                                br_oct <= 3'd0;
                                if (br_x <= br_y) begin
                                    br_x <= br_x + 11'd1;
                                    if (br_d[11])
                                        br_d <= br_d + {9'd0, br_x, 1'b0} + 12'd1;
                                    else begin
                                        br_y <= br_y - 11'd1;
                                        br_d <= br_d + {9'd0, br_x, 1'b0} - {9'd0, br_y, 1'b0} + 12'd1;
                                    end
                                    br_state <= 4'd2;
                                end else begin
                                    if (br_layer == 3'd2) begin
                                        br_state <= 4'd0;
                                    end else begin
                                        br_layer <= br_layer + 3'd1;
                                        br_rr <= br_rr + 7'd1;
                                        br_state <= 4'd1;
                                    end
                                end
                            end else begin
                                br_oct <= br_oct + 3'd1;
                                br_state <= 4'd2;
                            end
                        end
                        default: br_state <= 4'd0;
                    endcase
                end

                // ---------------------------------------------------------
                // S_WAIT_PASS2: 等待 DMA 第二次发帧
                // ---------------------------------------------------------
                S_WAIT_PASS2: begin
                    s_axis_tready <= 1'b1;
                    p2_pixel_idx <= 21'd0;
                    p2_valid     <= 1'b0;
                    if (s_axis_tvalid) begin
                        state <= S_PASS2_OVERLAY;
                    end
                end

                // ---------------------------------------------------------
                // S_PASS2_OVERLAY: 第二遍 — 叠加输出
                // ---------------------------------------------------------
                S_PASS2_OVERLAY: begin
                    s_axis_tready <= m_axis_tready;

                    // 流水线第0级: 计算位图地址
                    p2_word_addr <= p2_pixel_idx / 17'd9;
                    p2_bit_pos   <= p2_pixel_idx % 4'd9;
                    p2_orig_data <= s_axis_tdata;
                    p2_orig_valid <= s_axis_tvalid;
                    p2_orig_last  <= s_axis_tlast;

                    // 流水线第1级: 读位图 + 输出
                    if (p2_valid) begin
                        m_axis_tvalid <= 1'b1;
                        p2_ov_word <= overlay_bm[p2_word_addr];
                        // 若叠加位置位, 输出白色 (0xFF), 否则直通
                        if (p2_ov_word[p2_bit_pos])
                            m_axis_tdata <= 8'hFF;
                        else
                            m_axis_tdata <= p2_orig_data;
                        m_axis_tlast <= p2_orig_last;
                        if (p2_orig_last)
                            state <= S_DONE;
                    end

                    p2_valid <= p2_orig_valid;

                    if (s_axis_tvalid && m_axis_tready) begin
                        p2_pixel_idx <= p2_pixel_idx + 21'd1;
                    end
                end

                // ---------------------------------------------------------
                // S_DONE: 帧完成
                // ---------------------------------------------------------
                S_DONE: begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase

            // =============================================================
            // 累加器写 (在投票循环中)
            // =============================================================
            if (ac_wr_en) begin
                case (ac_r_idx)
                    4'd0: accum_0[ac_addr] <= ac_wr_data;
                    4'd1: accum_1[ac_addr] <= ac_wr_data;
                    4'd2: accum_2[ac_addr] <= ac_wr_data;
                    4'd3: accum_3[ac_addr] <= ac_wr_data;
                    4'd4: accum_4[ac_addr] <= ac_wr_data;
                    4'd5: accum_5[ac_addr] <= ac_wr_data;
                    4'd6: accum_6[ac_addr] <= ac_wr_data;
                    4'd7: accum_7[ac_addr] <= ac_wr_data;
                    4'd8: accum_8[ac_addr] <= ac_wr_data;
                    4'd9: accum_9[ac_addr] <= ac_wr_data;
                endcase
            end
        end
    end

    // =========================================================================
    // 累加器读 (组合逻辑选择 + 寄存器输出)
    // =========================================================================
    always @(posedge clk) begin
        case (ac_r_idx)
            4'd0: ac_rd_data <= accum_0[ac_addr];
            4'd1: ac_rd_data <= accum_1[ac_addr];
            4'd2: ac_rd_data <= accum_2[ac_addr];
            4'd3: ac_rd_data <= accum_3[ac_addr];
            4'd4: ac_rd_data <= accum_4[ac_addr];
            4'd5: ac_rd_data <= accum_5[ac_addr];
            4'd6: ac_rd_data <= accum_6[ac_addr];
            4'd7: ac_rd_data <= accum_7[ac_addr];
            4'd8: ac_rd_data <= accum_8[ac_addr];
            4'd9: ac_rd_data <= accum_9[ac_addr];
            default: ac_rd_data <= 9'd0;
        endcase
    end

endmodule
