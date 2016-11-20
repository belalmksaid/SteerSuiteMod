function h = partest(filename, color, runNum)
fileID = fopen(filename);
cellr = textscan(fileID, ['%f', repmat('%f ', [1, runNum]), '%*[^\n]']);
matr = cell2mat(cellr);
dr = 1:100;
sample = matr(dr,1).^2;
data = matr(dr, 2:end)';
if (runNum > 1)
    meanData = mean(data);
    stdData = std(data);
else
    meanData = data;
    stdData = 0;
end
h = plot(sample, meanData, ['-',color]);
hold on
grid on
plot(sample, meanData+2*stdData, ['--',color]);
plot(sample, meanData-2*stdData, ['--',color]);
fclose(fileID);
end