// top_level.v
// 功能：上电后自动向 0x78 发送 0xB8，send_done 拉高表示完成
// 时钟：50 MHz，I2C：100 kHz

module top_level (
    input  wire clk,
    input  wire rst_n,
    input  wire start_tx,   // 上升沿触发一次发送
    output wire send_done,  // 发送完成脉冲（1个时钟周期）
    inout  wire i2c_scl,
    inout  wire i2c_sda
);

// ----------------------------------------------------------------
// 参数
// ----------------------------------------------------------------
localparam SLAVE_ADDR = 7'h78;
localparam TX_DATA    = 8'hB8;
// prescale = Fclk / (FI2C * 4) = 50_000_000 / (100_000 * 4) = 125
localparam PRESCALE   = 16'd125;

// ----------------------------------------------------------------
// I2C 引脚连接（open-drain，不需要用 _t 信号）
// ----------------------------------------------------------------
wire scl_o, sda_o;
wire scl_i, sda_i;

assign i2c_scl = scl_o ? 1'bz : 1'b0;
assign i2c_sda = sda_o ? 1'bz : 1'b0;
assign scl_i   = i2c_scl;
assign sda_i   = i2c_sda;

// ----------------------------------------------------------------
// AXI-Stream 信号
// ----------------------------------------------------------------
reg        cmd_valid;
wire       cmd_ready;

reg  [7:0] tx_tdata;
reg        tx_tvalid;
reg        tx_tlast;
wire       tx_tready;

wire       i2c_busy;

// ----------------------------------------------------------------
// 状态机
// ----------------------------------------------------------------
localparam S_IDLE = 2'd0;
localparam S_CMD  = 2'd1;   // 发送命令（地址 + 读写标志）
localparam S_DATA = 2'd2;   // 发送数据字节
localparam S_WAIT = 2'd3;   // 等待 I2C 总线 busy 拉低

reg [1:0] state;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state     <= S_IDLE;
        cmd_valid <= 1'b0;
        tx_tvalid <= 1'b0;
    end else begin
        case (state)

            S_IDLE: begin
                if (start_tx) begin
                    cmd_valid <= 1'b1;   // 准备好命令，等待 master 接收
                    state     <= S_CMD;
                end
            end

            S_CMD: begin
                // AXI-Stream 握手：valid & ready 同时为高时，传输成功
                if (cmd_ready) begin
                    cmd_valid <= 1'b0;   // 命令已被接受
                    tx_tdata  <= TX_DATA;
                    tx_tvalid <= 1'b1;
                    tx_tlast  <= 1'b1;   // 只有1字节，这也是最后1字节
                    state     <= S_DATA;
                end
            end

            S_DATA: begin
                if (tx_tready) begin
                    tx_tvalid <= 1'b0;   // 数据已被接受
                    state     <= S_WAIT;
                end
            end

            S_WAIT: begin
                // 等待 i2c_master 完成总线传输（发完 STOP）
                if (!i2c_busy)
                    state <= S_IDLE;
            end

        endcase
    end
end

assign send_done = (state == S_WAIT) && !i2c_busy;  // 完成脉冲

// ----------------------------------------------------------------
// 实例化 i2c_master
// ----------------------------------------------------------------
i2c_master u_i2c_master (
    .clk                     (clk),
    .rst                     (~rst_n),

    // 命令通道
    .s_axis_cmd_address      (SLAVE_ADDR),
    .s_axis_cmd_start        (1'b1),       // 总是发 START
    .s_axis_cmd_read         (1'b0),       // 写操作，不读
    .s_axis_cmd_write        (1'b1),       // 写1字节
    .s_axis_cmd_write_multiple(1'b0),
    .s_axis_cmd_stop         (1'b1),       // 写完后发 STOP
    .s_axis_cmd_valid        (cmd_valid),
    .s_axis_cmd_ready        (cmd_ready),

    // 写数据通道
    .s_axis_data_tdata       (tx_tdata),
    .s_axis_data_tvalid      (tx_tvalid),
    .s_axis_data_tready      (tx_tready),
    .s_axis_data_tlast       (tx_tlast),

    // 读数据通道（本例不用，直接接受）
    .m_axis_data_tdata       (),
    .m_axis_data_tvalid      (),
    .m_axis_data_tready      (1'b1),
    .m_axis_data_tlast       (),

    // I2C 引脚
    .scl_i                   (scl_i),
    .scl_o                   (scl_o),
    .scl_t                   (),
    .sda_i                   (sda_i),
    .sda_o                   (sda_o),
    .sda_t                   (),

    // 状态
    .busy                    (i2c_busy),
    .bus_control             (),
    .bus_active              (),
    .missed_ack              (),

    // 配置
    .prescale                (PRESCALE),
    .stop_on_idle            (1'b1)
);

endmodule