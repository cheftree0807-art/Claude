module histogram_eq_top (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,
    input  wire  [7:0]  s_axis_tdata,
    input  wire         s_axis_tlast,

    output wire         m_axis_tvalid,
    input  wire         m_axis_tready,
    output wire  [7:0]  m_axis_tdata,
    output wire         m_axis_tlast
);

    localparam GRAY_LEVELS = 256;

    localparam S_IDLE      = 3'd0;
    localparam S_HIST_RD   = 3'd1;
    localparam S_HIST_WR   = 3'd2;
    localparam S_CDF       = 3'd3;
    localparam S_MAP       = 3'd4;
    localparam S_MAP_CALC  = 3'd5;
    localparam S_PROC      = 3'd6;
    localparam S_LAST      = 3'd7;

    reg  [2:0] state;
    reg [7:0]  gray_map [0:GRAY_LEVELS-1];
    (* ram_style = "block" *) reg [31:0] histogram [0:GRAY_LEVELS-1];
    (* ram_style = "block" *) reg [31:0] cdf      [0:GRAY_LEVELS-1];
    reg [31:0] total_pixels;
    reg [31:0] pixel_cnt;
    reg [8:0]  addr_cnt;
    reg [7:0]  hist_addr_d;          // 直方图地址延迟1拍（读BRAM需要）
    reg        hist_last_d;          // tlast延迟1拍

    // 流水线寄存器
    reg [31:0] hist_rd;              // 直方图读取值
    reg [8:0]  cdf_addr_a, cdf_addr_b;
    reg [31:0] cdf_prev;
    reg [31:0] hist_val;

    // 除法器改进：直接用乘法调用DSP48即可，流水线化
    reg [31:0] dividend, divisor, quotient, remainder;
    reg [5:0]  div_bit;
    reg        div_busy;

    // 输出寄存器
    reg        m_axis_tvalid_r;
    reg [7:0]  m_axis_tdata_r;
    reg        m_axis_tlast_r;

    wire [39:0] mult_prod;
    wire [31:0] current_cdf;
    assign current_cdf = cdf[addr_cnt];
    assign mult_prod    = current_cdf * 32'd255;

    // 乘法流水线寄存器（DSP48输出寄存）
    reg [39:0] mult_prod_reg;

    // 除法移位后的余数（组合逻辑，避免非阻塞赋值比较错误）
    wire [31:0] remainder_shifted;
    assign remainder_shifted = {remainder[30:0], dividend[31]};

    integer i;

    // ===================== 主状态机 =====================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            pixel_cnt       <= 32'd0;
            addr_cnt        <= 9'd0;
            hist_addr_d     <= 8'd0;
            hist_last_d     <= 1'b0;
            total_pixels    <= 32'd0;
            m_axis_tvalid_r <= 1'b0;
            m_axis_tdata_r  <= 8'd0;
            m_axis_tlast_r  <= 1'b0;

            hist_rd         <= 32'd0;
            cdf_addr_a      <= 9'd0;
            cdf_addr_b      <= 9'd0;
            cdf_prev        <= 32'd0;
            hist_val        <= 32'd0;

            dividend        <= 32'd0;
            divisor         <= 32'd0;
            quotient        <= 32'd0;
            remainder       <= 32'd0;
            div_bit         <= 6'd0;
            div_busy        <= 1'b0;

            mult_prod_reg   <= 40'd0;

            for (i = 0; i < GRAY_LEVELS; i = i + 1) begin
                histogram[i]  <= 32'd0;
                cdf[i]        <= 32'd0;
                gray_map[i]   <= 8'd0;
            end
        end else begin
            case (state)
                // ------------------------------------------------------------
                S_IDLE: begin
                    addr_cnt        <= 9'd0;
                    m_axis_tvalid_r <= 1'b0;
                    m_axis_tlast_r  <= 1'b0;
                    if (s_axis_tvalid) begin
                        state       <= S_HIST_RD;
                        pixel_cnt   <= 32'd0;
                        hist_addr_d <= s_axis_tdata;   // 记录地址
                        for (i = 0; i < GRAY_LEVELS; i = i + 1)
                            histogram[i] <= 32'd0;
                    end
                end

                // ------------------------------------------------------------
                // 直方图统计：读阶段（BRAM需要1拍读取延迟）
                S_HIST_RD: begin
                    hist_rd      <= histogram[s_axis_tdata];
                    hist_addr_d  <= s_axis_tdata;
                    hist_last_d  <= s_axis_tlast;
                    state        <= S_HIST_WR;
                end

                // ------------------------------------------------------------
                // 直方图统计：写阶段（读延迟1拍后写入）
                S_HIST_WR: begin
                    histogram[hist_addr_d] <= hist_rd + 1'b1;
                    pixel_cnt              <= pixel_cnt + 1'b1;

                    if (hist_last_d) begin
                        total_pixels <= pixel_cnt + 1'b1;
                        state        <= S_CDF;
                        addr_cnt     <= 9'd0;
                    end else begin
                        state        <= S_HIST_RD;
                        hist_addr_d  <= s_axis_tdata;
                    end
                end

                // ------------------------------------------------------------
                // CDF计算
                S_CDF: begin
                    if (addr_cnt == 9'd0) begin
                        cdf[0] <= histogram[0];
                    end else begin
                        cdf[addr_cnt] <= cdf[addr_cnt-1] + histogram[addr_cnt];
                    end

                    if (addr_cnt == 9'd255) begin
                        state    <= S_MAP;
                        addr_cnt <= 9'd0;
                    end
                    addr_cnt <= addr_cnt + 1'b1;
                end

                // ------------------------------------------------------------
                // 启动除法：DSP48乘法 → 流水线寄存器
                S_MAP: begin
                    mult_prod_reg <= mult_prod;
                    state         <= S_MAP_CALC;
                end

                // ------------------------------------------------------------
                // 除法计算
                S_MAP_CALC: begin
                    if (!div_busy) begin
                        dividend  <= mult_prod_reg[31:0];
                        divisor   <= total_pixels;
                        quotient  <= 32'd0;
                        remainder <= 32'd0;
                        div_bit   <= 6'd31;
                        div_busy  <= 1'b1;
                    end else begin
                        dividend <= dividend << 1;

                        if (remainder_shifted >= divisor) begin
                            quotient[div_bit] <= 1'b1;
                            remainder <= remainder_shifted - divisor;
                        end else begin
                            quotient[div_bit] <= 1'b0;
                            remainder <= remainder_shifted;
                        end

                        if (div_bit == 6'd0) begin
                            div_busy <= 1'b0;
                            gray_map[addr_cnt] <= quotient[7:0];
                            if (addr_cnt == 9'd255) begin
                                state    <= S_PROC;
                                addr_cnt <= 9'd0;
                            end else begin
                                addr_cnt <= addr_cnt + 1'b1;
                                state    <= S_MAP;
                            end
                        end else begin
                            div_bit <= div_bit - 1'b1;
                        end
                    end
                end

                // ------------------------------------------------------------
                // 应用灰度映射
                S_PROC: begin
                    if (s_axis_tvalid && m_axis_tready) begin
                        m_axis_tdata_r  <= gray_map[s_axis_tdata];
                        m_axis_tvalid_r <= 1'b1;
                        m_axis_tlast_r  <= s_axis_tlast;
                        if (s_axis_tlast)
                            state <= S_LAST;
                    end else begin
                        m_axis_tvalid_r <= 1'b0;
                    end
                end

                // ------------------------------------------------------------
                S_LAST: begin
                    state           <= S_IDLE;
                    m_axis_tvalid_r <= 1'b0;
                    m_axis_tlast_r  <= 1'b0;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // ready信号：S_IDLE也需要准备好接收数据
    assign s_axis_tready = (state == S_IDLE) || (state == S_HIST_RD)
                        || (state == S_PROC && m_axis_tready);

    assign m_axis_tvalid = m_axis_tvalid_r;
    assign m_axis_tdata  = m_axis_tdata_r;
    assign m_axis_tlast  = m_axis_tlast_r;

endmodule