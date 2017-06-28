function inters = solveIntersection(x1,x2,y1,y2,cx,cy,r)
x = [x1 x2];
y = [y1 y2];
c = [[1;1] x(:)]\y(:);
slope = c(2);
intercept = c(1);
[x,y] = linecirc(slope,intercept,cx,cy,r);
inters = [x(1),y(1);x(2),y(2)];
end