clear;
clc;
syms th1 th2 th3 th4 th5 ...
    a1  a2  a3  a4  a5 ...
    d1  d5...
    t11 t12 t13 t14 t21 t22 t23 t24 t31 t32 t33 t34

%{
DH = [-0.033, 0.155, 0.135,  0.002,      0;...  % a
      pi/2 ,     0 ,     0,   pi/2,     pi;...  % alpha
      0.145,      0,     0,      0, -0.185; ... % d
      pi,      pi/2,     0,  -pi/2,     pi];    % theta
%}
DH_params = [a1,      a2,    a3,     a4,     a5;...  % a
      pi/2,     0,     0,   pi/2,     pi;...  % alpha
      d1,       0,     0,      0,     d5; ... % d
      pi,    pi/2,     0,  -pi/2,     pi];    % theta

theta = [th1 th2 th3 th4 th5];

T0e = eye(4);

for i = 1:5
    theta = DH_params(4,i) + theta(i,:);
    alpha = DH_params(2,i);
    a  = DH_params(1,i);
    d  = DH_params(3,i);
           
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);...
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);...
         0,       sin(alpha),         cos(alpha),          d;...
         0,       0,               0,                1];
    T0e = T0e * T;
end

for i = 1:4
    for j = 1:4
        T0e(i,j) = simplify (T0e(i,j));
        disp([i,j])
        disp(T0e(i,j));
    end
end

T14 = T0e(1,4);
T24 = T0e(2,4);
T34 = T0e(3,4);
a5 = (-cos(th1) * T24 + sin(th1) * T14) / sin(th5);

[th1s, th5s] = solve([a5 T34], [th1, th2])



