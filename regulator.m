clc
clear all

tug=535;

k=0.05;
u=zeros(1,1000);
CCR=zeros(1,1000);
%//Генеририруем уставку
ust=zeros(1,1000);

for m=1:100
   ust(m)=500;
end
for m=100:1000
   ust(m)=3000;
end
CCR(1) = 666;
u(1) = -0.0001*(CCR(1))^3 + 0.1231*(CCR(1))^2 - 52.815*CCR(1) + 10124;
for i=1:1000
    CCR(i+1)=CCR(i)-k*(ust(i)-u(i));
    u(i+1) = -0.0001*(CCR(i+1))^3 + 0.1231*(CCR(i+1))^2 - 52.815*CCR(i+1) + 10124;
end

plot(ust);
hold on;
plot(CCR);
hold on;
plot(u);
