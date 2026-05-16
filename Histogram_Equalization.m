% ======================
% 全局直方图均衡化去雾
% ======================
function out_img = Histogram_Equalization(gray_img)
    [height, width] = size(gray_img);
    total_pixels = height * width;

    % 1. 统计灰度直方图
    hist = zeros(256, 1);
    for i = 1:height
        for j = 1:width
            val = gray_img(i,j) + 1; 
            hist(val) = hist(val) + 1;
        end
    end

    % 2. 计算累积分布函数 CDF
    cdf = zeros(256, 1);
    cdf(1) = hist(1) / total_pixels;
    for k = 2:256
        cdf(k) = cdf(k-1) + hist(k) / total_pixels;
    end

    % 3. 灰度映射
    out_img = zeros(height, width, 'uint8');
    for i = 1:height
        for j = 1:width
            old_val = gray_img(i,j);
            new_val = round(255 * cdf(old_val + 1));
            out_img(i,j) = new_val;
        end
    end
end