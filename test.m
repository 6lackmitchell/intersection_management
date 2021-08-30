x = 1:10;
y = 5*rand(size(x)) + 2.5;
[marker,m] = imread('bmw.png');
marker = imresize(marker,0.5);
markersize = [1,1]; %//The size of marker is expressed in axis units, NOT in pixels
x_low = x - markersize(1)/2; %//Left edge of marker
x_high = x + markersize(1)/2;%//Right edge of marker
y_low = y - markersize(2)/2; %//Bottom edge of marker
y_high = y + markersize(2)/2;%//Top edge of marker
figure();
hold on
for k = 1:length(x)
    [x_low(k) x_high(k)] = rotate([0 1; -1 0],[x_low(k) x_high(k)]);
    [y_low(k) y_high(k)] = rotate([0 1; -1 0],[y_low(k) y_high(k)]);
    imagesc([x_low(k) x_high(k)], [y_low(k) y_high(k)],marker)
end
% axis equal
hold off

function [newx_coord,newy_coord] = rotate(matrix,vector)
    z          = vector*matrix;
    newx_coord = z(1);
    newy_coord = z(2);
end