clear all;
%close all;
numOffsetSamples = 100;
for i = 1:numOffsetSamples
%% setup sensor accuracy
a = 4417.7766959627;
b = -66.1650874647;
std = 69.03631579;
%% setup robot params
xs = 4.5; %inch
ys = 3;
%% setup sensor readings
d_1 = 3.5;%3.5; %inch
d_2 = 3.5;%3.5; %inch
numSamplesToAvg = 16;%4^2;
d_1_total = 0;
d_2_total = 0;
for j=1:numSamplesToAvg
  %%compute Sensor Error - using avg
  val_1 = ((1/((d_1*2.54)+.42))*a) - b;
  val_1 = normrnd(val_1,std,1,1);
  val_2 = ((1/((d_2*2.54)+.42))*a) - b;
  val_2 = normrnd(val_2,std,1,1);
  d_1_total = d_1_total+val_1;
  d_2_total = d_2_total+val_2;
end
val_1 = d_1_total/numSamplesToAvg;
val_2 = d_2_total/numSamplesToAvg;
d_2 = (val_2+b)/a;
d_2 = ((1/d_2)-.42)/2.54;
d_1 = (val_1+b)/a;
d_1 = ((1/d_1)-.42)/2.54;
%%compute offset
heading = (pi()/2) - (atan((2*ys)/abs(d_2-d_1)));
x_offset(i) = (cos(atan(ys/xs)-heading)*sqrt((xs^2) + (ys^2)))+(cos(heading)*(d_1+d_2)/2);
end
figure
plot(x_offset);
axis([1 numOffsetSamples 8-2 8+2])
