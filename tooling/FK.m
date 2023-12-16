function [fx,fy,fz,T] = FK(c,joint)
alp2=c(1);
a2 = c(2);
a3 = c(3);
a4 = c(4);
t1 = joint(1);
t2 = joint(2);
t3 = joint(3);
t4 = joint(4);


% DH parameters
DH = [0 0 0 t1;
      alp2 0 0 t2;
      0 a2 0 t3;
      0 a3 0 t4;
      0 a4 0 0];
alpha = DH(:,1); a = DH(:,2); d = DH(:,3); theta = DH(:,4);

% initial
To = eye(4);
fx = 0; fy = 0; fz = 0;
T{1} = To;

for j = 1:5
   

    Ti = [cos(theta(j)) -sin(theta(j)) 0 a(j);
        sin(theta(j))*cos(alpha(j)) cos(theta(j))*cos(alpha(j)) ...
        -sin(alpha(j)) -sin(alpha(j))*d(j);
        sin(theta(j))*sin(alpha(j)) cos(theta(j))*sin(alpha(j)) ...
        cos(alpha(j)) cos(alpha(j))*d(j);0 0 0 1];
    To = To*Ti;
    fx = [fx To(1,4)]; % frame x coordiante
    fy = [fy To(2,4)]; % frame y coordiante
    fz = [fz To(3,4)]; % frame z coordiante
    T{j+1} = To;
    
end

end