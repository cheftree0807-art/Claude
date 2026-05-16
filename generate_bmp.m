% 创建1280x720的灰度图像
width = 1280;
height = 720;

% 生成简单的渐变灰度图案
[x, y] = meshgrid(1:width, 1:height);
img = uint8(mod((x + y) / 2, 256));

% 保存为BMP文件（8位灰度）
imwrite(img, 'test_1280x720_gray.bmp');

% 显示图像信息
disp('BMP文件已生成: test_1280x720_gray.bmp');
disp(['图像分辨率: ', num2str(width), 'x', num2str(height)]);
disp(['图像大小: ', num2str(width * height), ' 字节']);
