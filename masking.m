left = imread('image.png');
right = imread('imager.png');

left_g = rgb2gray(left);
right_g = rgb2gray(right);

l_m = left_g < 100 ;
r_m = right_g < 100 ;

l_m = l_m*255;
l_m = 255-l_m;

r_m = r_m*255;
r_m = 255-r_m;

imwrite(l_m, 'left_masking.jpg');
imwrite(r_m, 'right_masking.jpg');

lr_m = l_m & r_m;
mask=lr_m*255;

imwrite(mask, 'mask_img.jpg');