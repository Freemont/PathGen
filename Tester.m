filename = 'Path1.txt';

clf('reset')
delimiterIn = ',';
headerlinesIn = 0;

A = importdata(filename,delimiterIn,headerlinesIn);
b= 13;
plot(A(1:1:length(A),1),A(1:1:length(A),2));
hold on;
plot(A(1:1:length(A),3),A(1:1:length(A),4));
hold on;
plot(A(1:1:length(A),5),A(1:1:length(A),6));