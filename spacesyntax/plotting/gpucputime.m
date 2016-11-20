h1 = partest('gpufinedata_time.txt', 'g', 10);
h2 = partest('dataoutputOMP3.txt', 'm', 10);
h3 = partest('0CUDA_time.txt', 'b', 10);
legend([h1, h2, h3], 'GPU-new', 'CPU-12@3.5GHz', 'GPU-K620-classic');
xlabel('Number of Nodes');
ylabel('Time (ms)');
title('GPU-CPU comparison');