# 更新Block Design的Tcl脚本

# 打开项目
open_project HE.xpr

# 打开Block Design
open_bd_design "HE.srcs/sources_1/bd/HE/HE.bd"

# 刷新模块定义
update_module [get_bd_cells histogram_eq_top_0]

# 验证设计
validate_bd_design

# 生成输出产品
generate_target all [get_files HE.srcs/sources_1/bd/HE/HE.bd]

# 创建HDL包装器
make_wrapper -files [get_files HE.srcs/sources_1/bd/HE/HE.bd] -top

# 关闭项目
close_project

puts "Block Design更新完成！"
