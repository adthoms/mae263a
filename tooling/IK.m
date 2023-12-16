function [t1,t2,t3,t4] = IK(T0e, phi ,c)

a2 = double(c(2));
a3 = double(c(3));
a4 = double(c(4));

T04 = T0e;
x = T04(1,4);
y = T04(2,4);
z = T04(3,4);

t1 = atan2(y,x);

c1 = cos(t1);
cPhi=cos(phi);
sPhi=sin(phi);
r2=(x/c1-a4*cPhi)^2+(z-a4*sPhi)^2;

c3=(r2-a2^2-a3^2)/(2*a2*a3);
s3=-sqrt(1-c3^2);

t3=atan2(s3,c3);

alpha=atan2(z-a4*sPhi,x/c1-a4*cPhi);

cBeta=(-a3^2+a2^2+r2)/(2*a2*sqrt(r2));
sBeta=sqrt(1-cBeta^2);
beta=atan2(sBeta,cBeta);

t2=alpha+beta;

t4=phi-t2-t3;
end