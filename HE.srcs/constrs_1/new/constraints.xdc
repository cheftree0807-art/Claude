# ==============================
# 最终零报错约束
# 只留这一行，其他全删
# ==============================
create_clock -name clk_100MHz -period 10 [get_ports FIXED_IO_ps_clk]