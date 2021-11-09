function [x, y, theta,isLocWorking,red_centroid,blue_centroid] = LocalizationTopView(current_frame)
% Function to localize the magnetic robot

% TODO: The following gives a example to find two a red region and a blue region
% Use your designed localization method to localize your robot. You can
% change the output to names as well. 

% output example: 
%   x: x coordinate of the robot in pixel coordinate
%   y: y coordinate of the robot in pixel coordinate
%   theta: orientation of the robot
%   isLocWorking: boolean showing if localization is working or not
%   red_centroid: centroid of red region 
%   blue_centroid: centroid of blue region
              
img_hsv = rgb2hsv(current_frame);
% img_slice = imcrop(img_hsv); % comment out later
% img_hsv = img_slice; % comment out later


rHmin = 0.85;
rHmax = 1.0;
rSmin = 0.5;
rSmax = 1.0;
rVmin = 0.7;
rVmax = 1.0;

bHmin = 0.4;
bHmax = 0.8;
bSmin = 0.5;
bSmax = 1.0;
bVmin = 0.5;
bVmax = 1.0;

r_thresh_img = img_hsv(:,:,1) >= rHmin...
    & img_hsv(:,:,1) <= rHmax...
    & img_hsv(:,:,2) >= rSmin...
    & img_hsv(:,:,2) <= rSmax...
    & img_hsv(:,:,3) >= rVmin...
    & img_hsv(:,:,3) <= rVmax;

b_thresh_img = img_hsv(:,:,1) >= bHmin...
    & img_hsv(:,:,1) <= bHmax...
    & img_hsv(:,:,2) >= bSmin...
    & img_hsv(:,:,2) <= bSmax...
    & img_hsv(:,:,3) >= bVmin...
    & img_hsv(:,:,3) <= bVmax;

r_filt_img = medfilt2(r_thresh_img,[3 3]);
b_filt_img = medfilt2(b_thresh_img,[3 3]);

r_dilate = strel('square',10);
b_dilate = strel('square',10);
r_erode = strel('square',5);
b_erode = strel('square',5);

r_dilated_img = imdilate(r_filt_img,r_dilate);
r_eroded_img = imerode(r_dilated_img,r_erode);
r_eroded_img = imdilate(r_eroded_img,r_dilate);


b_dilated_img = imdilate(b_filt_img,b_dilate);
b_eroded_img = imerode(b_dilated_img,b_erode);
b_eroded_img = imdilate(b_eroded_img,r_dilate);

r_bw_conncomp = bwconncomp(r_eroded_img);
r_stats = regionprops(r_bw_conncomp,'Area','Centroid','Eccentricity');
r_idx = find([r_stats.Area] > 700 & [r_stats.Eccentricity] < 1.0);
r_blobs = ismember(labelmatrix(r_bw_conncomp), r_idx);
r_blobs = bwconncomp(r_blobs);
r_stats = regionprops(r_blobs,'Area','Centroid','Eccentricity');

b_bw_conncomp = bwconncomp(b_eroded_img);
b_stats = regionprops(b_bw_conncomp,'Area','Centroid','Eccentricity');
b_idx = find([b_stats.Area] > 700 & [b_stats.Eccentricity] < 1.0);
b_blobs = ismember(labelmatrix(b_bw_conncomp), b_idx);
% imshow(b_blobs)
b_blobs = bwconncomp(b_blobs);
b_stats = regionprops(b_blobs,'Area','Centroid','Eccentricity');


r_x = r_stats(1).Centroid(1);
r_y = r_stats(1).Centroid(2);


b_x = b_stats(1).Centroid(1);
b_y = b_stats(1).Centroid(2);

x = (r_x + b_x)/2;

y = (r_y + b_y)/2;

theta = atan2((r_y - b_y),(r_x - b_x));

red_centroid = r_stats.Centroid;
blue_centroid = b_stats.Centroid;

isLocWorking = 1;

if length(r_stats) < 1 || length(b_stats) < 1
    x = 0;
    y = 0;
    theta = 0;
    red_centroid = [0,0];
    blue_centroid = [0,0];
    disp("Localization Failed")
    isLocWorking = 0;
end

end



