function motion_detection(filename)
if nargin < 1
    filename = 'moving_objects2.mp4';
end
A = VideoReader(filename);
map = [(0:255)' (0:255)' (0:255)']/255;
mov(1:A.NumberOfFrames) = struct('cdata', ...
    zeros(A.Height, A.Width, 3, 'uint8'), 'colormap', map);
for k = 1:A.NumberOfFrames
    frame = read(A, k);
    mov(k).cdata(:,:,1) = rgb2gray(frame);
    mov(k).cdata(:,:,2) = rgb2gray(frame);
    mov(k).cdata(:,:,3) = rgb2gray(frame);
end

difference(1:A.NumberOfFrames-1) = struct('cdata', ...
    zeros(A.Height, A.Width, 1, 'uint8'), 'colormap', map);

% KALMAN INITIAL VALUES
xpred = [A.Height/2 A.Width/2 0 0]';
xcor = xpred;
sigpred = eye(4);
sigcor = sigpred;

D = [1 0 1/A.FrameRate 0; 0 1 0 1/A.FrameRate; 0 0 1 0; 0 0 0 1];
M = [1 0 0 0; 0 1 0 0];
sigd = eye(4);
sigy = eye(2);

u(1) = A.Height/2;
u(2) = A.Width/2;

for k = 1:A.NumberOfFrames-1
    difference(k).cdata = medfilt2(255*uint8(abs(mov(k+1).cdata(:,:,1) - mov(k).cdata(:,:,1)) > 30));
    stats = regionprops(difference(k).cdata);
    if ~isempty(stats) % new data: centroid, bounding box
        u = stats(255).Centroid;
        u = [u(2) u(1)];
        Ki = sigpred*M'/(M*sigpred*M'+sigy);
        xcor = xpred + Ki*(u' - M*xpred);
        sigcor = (eye(4) - Ki*M)*sigpred;
    end
%     mov(k+1).cdata = boundingbox(mov(k).cdata, stats(255).BoundingBox, white);
    xpred = D*xcor;
    mov(k+1).cdata = boundingbox(mov(k+1).cdata, [u-1 2 2], [255 0 0]); % actual
    mov(k+1).cdata = boundingbox(mov(k+1).cdata, [xpred(1:2)'-1 2 2], [0 255 0]);
    sigpred = sigd + D*sigcor*D;
end
play_movie(A, mov);
end

function output = boundingbox(mat, stuff, color)
[r, c, depth] = size(mat);
output = mat;
rowstart = round(stuff(1));
colstart = round(stuff(2));
rowdist = stuff(3);
coldist = stuff(4);
output(rowstart:rowstart + rowdist, colstart, :) = repmat(reshape(color, 1, 1, 3), rowdist+1, 1);
output(rowstart:rowstart + rowdist, colstart + coldist, :) = repmat(reshape(color, 1, 1, 3), rowdist+1, 1);
output(rowstart, colstart:colstart + coldist, :) = repmat(reshape(color, 1, 1, 3), 1, coldist+1);
output(rowstart + rowdist, colstart:colstart + coldist, :) = repmat(reshape(color, 1, 1, 3), 1, coldist+1);
if size(output) ~= [r, c, depth]
    output = output(1:r, 1:c, :);
end
end
function play_movie(movie_object, stuff)
hf = figure;
set(hf, 'position', [10 50 movie_object.Width movie_object.Height]);
movie(hf, stuff, 1, movie_object.FrameRate);
end