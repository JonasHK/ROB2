function [center, objectcenter, majoraxis] = findGreenCircle(imsub)
%Find green circle in image
	% Receive RGB-image from the turtlebots camera.
	% Removing the blue and red components of the image, the function find
	% green components in the image. Using morphological methods (regionprops)
	% it returns information about circles in the image.

close all;
% Receive RGB-image
imgraw = receive(imsub); % a serialised image
img = readImage(imgraw); % decode image

% Create grayscale image, by removing the red and blue components of the
% image.
imR = img(:,:,1); % red channel
imG = img(:,:,2); % green channel
imB = img(:,:,3); % blue channel
I = 2*imG - (imB+imR);

% Use thresholding to convert the grayscale image to a binary image.
threshold = 150;
img_bin = zeros(size(I));
thres = I>threshold;
img_bin(thres)=255;
img_bin = img_bin > 0;


%Opening - Not necessary in simulation, due to no noise
SE = ones(5); %Structuring Element
img_bin_open = imopen(img_bin,SE); %Opening



% finding properties of connected components.
myStats = regionprops(img_bin_open, 'Centroid', 'MajoraxisLength'); 

% If no objects are found by regionprops, set the output to zero, to avoid
% error.
% Otherwise set the output values to the regionprops.
if size(myStats, 1) == 0
	center = size(img,2)/2;
	objectcenter = 0;
	majoraxis = 0;
else
	myCenter = myStats(1).Centroid;
	myMajorAxis = myStats(1).MajorAxisLength;

	objectcenter = myCenter(1);
	center = size(img,2)/2;
	majoraxis = myMajorAxis;
end

end

