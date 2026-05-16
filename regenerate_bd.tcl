# 重新生成Block Design的Tcl脚本

# 打开项目
open_project HE.xpr

# 打开Block Design
open_bd_design "HE.srcs/sources_1/bd/HE/HE.bd"

# 刷新模块定义
update_ip_catalog -rebuild

# 更新模块
update_module [get_bd_cells hough_circle_top_0]

# 验证设计
validate_bd_design

# 生成输出产品
generate_target all [get_files HE.srcs/sources_1/bd/HE/HE.bd]

# 创建HDL包装器
make_wrapper -files [get_files HE.srcs/sources_1/bd/HE/HE.bd] -top

# 更新编译顺序
update_compile_order -fileset sources_1

# 关闭项目
close_project

puts "Block Design重新生成完成！"
