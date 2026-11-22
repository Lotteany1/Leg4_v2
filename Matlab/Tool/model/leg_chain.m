function [pos,J] = leg_chain(phi1,phi4)
%#codegen
% 001 双闭环：输入保留安装标定的两个电机角坐标，输出 [腿长;极角]。
% 后电机仅在 CAD 零位与虚拟曲柄重合，运动及虚功经真实传动计算。
coder.inline('always');
g = cast(leg_mechanism_data(),'like',phi1);
zero1 = atan2(-g(2,2),g(1,2));
zero4 = atan2(-g(2,2),-g(1,2));
a = [zero1-phi1;zero4-phi4];
B = rotate(a(1),g(:,2)); D = rotate(a(2),g(:,4)); P = rotate(a(1),g(:,7));
JB = zeros(2,2,'like',phi1); JD = JB; JP = JB;
JB(:,1) = -perp(B); JD(:,2) = -perp(D); JP(:,1) = -perp(P);
[E,JE] = circle(D,P,g(:,4),g(:,7),g(:,5),JD,JP);
rock = signed_angle(g(:,5)-g(:,7),E-P);
F = E+rotate(rock,g(:,6)-g(:,5));
drock = (perp(E-P)'*(JE-JP))/sum((E-P).^2);
JF = JE+perp(F-E)*drock;
[pin,Jpin] = circle(B,F,g(:,2),g(:,6),g(:,8),JB,JF);
shank = signed_angle(g(:,8)-g(:,2),pin-B);
C = B+rotate(shank,g(:,9)-g(:,2));
dshank = (perp(pin-B)'*(Jpin-JB))/sum((pin-B).^2);
JC = JB+perp(C-B)*dshank;
xy = [C(1);-C(2)]; Jxy = [JC(1,:);-JC(2,:)];
L = sqrt(sum(xy.^2));
pos = [L;atan2(xy(2),xy(1))];
J = [xy'/L;perp(xy)'/(L*L)]*Jxy;
end

function out = rotate(a,v)
coder.inline('always');
c = cos(a); s = sin(a);
out = [c*v(1)-s*v(2);s*v(1)+c*v(2)];
end

function out = perp(v)
coder.inline('always');
out = [-v(2);v(1)];
end

function a = signed_angle(v,w)
coder.inline('always');
a = atan2(v(1)*w(2)-v(2)*w(1),v'*w);
end

function [p,Jp] = circle(a,b,a0,b0,p0,Ja,Jb)
% 同一装配支路的圆交点；隐式微分两条等长约束得到解析雅可比。
coder.inline('always');
ra2 = sum((p0-a0).^2); rb2 = sum((p0-b0).^2);
delta = b-a; d = sqrt(sum(delta.^2));
t = (ra2-rb2+d*d)/(2*d); height2 = ra2-t*t;
if height2<=0 || d<=0
    p = nan(2,1,'like',a); Jp = nan(2,2,'like',a);
    return;
end
u = delta/d;
branch = sign((b0(1)-a0(1))*(p0(2)-a0(2))-(b0(2)-a0(2))*(p0(1)-a0(1)));
p = a+t*u+branch*sqrt(height2)*perp(u);
v = p-a; w = p-b;
rhs = [v'*Ja;w'*Jb]; determinant = v(1)*w(2)-v(2)*w(1);
Jp = [w(2)*rhs(1,:)-v(2)*rhs(2,:);-w(1)*rhs(1,:)+v(1)*rhs(2,:)]/determinant;
end
