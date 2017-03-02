function Points = load3dPoints(filename)
f = fopen(filename, 'r');
length = fread(f, 1, 'uint32');
Points = zeros([length 3], 'single');
for i=1:length
    Points(i,1:3) = fread(f, 3, 'single');
end

