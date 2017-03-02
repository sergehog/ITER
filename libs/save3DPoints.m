function save3DPoints(filename, Points)
length = size(Points, 1);
f = fopen(filename, 'w');
fwrite(f, length, 'uint32');
for i=1:length
    fwrite(f, Points(i,1), 'single');
    fwrite(f, Points(i,2), 'single');
    fwrite(f, Points(i,3), 'single');
end
fclose(f);
