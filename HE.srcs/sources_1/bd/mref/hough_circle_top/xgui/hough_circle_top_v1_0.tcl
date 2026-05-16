# hough_circle_top GUI 配置
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  ipgui::add_page $IPINST -name "Page 0"
}
