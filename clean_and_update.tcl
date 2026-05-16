# 清理和更新Block Design的Tcl脚本

# 打开项目
open_project HE.xpr

# 清理项目
reset_run synth_1
reset_run impl_1

# 打开Block Design
open_bd_design "HE.srcs/sources_1/bd/HE/HE.bd"

# 刷新模块定义
update_ip_catalog -rebuild

# 验证设计
validate_bd_design

# 重新生成Block Design
regenerate_bd_layout

# 生成输出产品
generate_target all [get_files HE.srcs/sources_1/bd/HE/HE.bd]

# 创建HDL包装器
make_wrapper -files [get_files HE.srcs/sources_1/bd/HE/HE.bd] -top

# 关闭项目
close_project

puts "项目清理和更新完成！"
