clear all;
close all;
numOffsetSamples = 100;
x_offset_total = 0;
x_offset_total_two = 0;
for i = 1:numOffsetSamples
%% setup sensor accuracy
a = 4417.7766959627;
b = -66.1650874647;
std = 37.03631579;
%% setup robot params
xs = 4.5; %inch
ys = 3;
%% setup sensor readings
d_1 = 10;%3.5;%3.5; %inch
d_2 = 3;%3.5; %inch
d_1_total = 0;
d_2_total = 0;
val_1 = ((1/((d_1*2.54)+.42))*a) - b;
val_1 = normrnd(val_1,std,1,1);
val_2 = ((1/((d_2*2.54)+.42))*a) - b;
val_2 = normrnd(val_2,std,1,1);
d_2 = (val_2+b)/a;
d_2 = ((1/d_2)-.42)/2.54;
d_1 = (val_1+b)/a;
d_1 = ((1/d_1)-.42)/2.54;
d_1_out(i) = d_1;
d_2_out(i) = d_2;
%%compute offset
x_offset_total = x_offset_total+ ((2*ys)/sqrt((2*ys)^2+(d_2-d_1)^2))*(((d_2+d_1)/2)+xs);
x_offset(i) = x_offset_total/i;
end
figure
plot(x_offset);
axis([1 numOffsetSamples x_offset(numOffsetSamples)-2 x_offset(numOffsetSamples)+2])
figure
plot(d_1_out,'*');
hold on;
plot(10*ones(1,numOffsetSamples));
figure
plot(d_2_out,'*');
hold on;
plot(3*ones(1,numOffsetSamples));
