filename = 'topkek.txt';

clf('reset')
delimiterIn = ',';
headerlinesIn = 1;

A = importdata(filename,delimiterIn,headerlinesIn);
plot(A.data(1:1:length(A.data),1),A.data(1:1:length(A.data),2));
hold on;
plot(A.data(1:1:length(A.data),3),A.data(1:1:length(A.data),4));
hold on;
plot(A.data(1:1:length(A.data),5),A.data(1:1:length(A.data),6));