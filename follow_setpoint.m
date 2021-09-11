function follow_setpoint = follow_setpoint(d1, d2)
    % c
    r = 0.6;
    a = (d2(1) - d1(1))^2 + (d2(2) - d1(2))^2 + (d2(3) - d1(3))^2 ;
    b = -2*((d2(1)- d1(1))*(d1(1)-d1(1))+(d2(2)- d1(2))*(d1(2)-d1(2))+(d2(3)- d1(3))*(d1(3)-d1(3)));
    c = (d1(1) - d1(1))^2 + (d1(2) - d1(2))^2 + (d1(2) - d1(2))^2 - r^2;
    p = [a b c];
    root = roots(p);
    disp("root");
    disp(d2)
    % 2 Possible points
    x1 = d1(1) + (d2(1) - d1(1))*root(1);
    y1 = d1(2) + (d2(2) - d1(2))*root(1);
    z1 = d1(3) + (d2(3) - d1(3))*root(1);
    
    x2 = d1(1) + (d2(1) - d1(1))*root(2);
    y2 = d1(2) + (d2(2) - d1(2))*root(2);
    z2 = d1(3) + (d2(3) - d1(3))*root(2);
    
    % choose root in between. criteria: the point closer to drone 2.

    if (norm(d2 - [x1; y1; z1]) < norm(d1 - [x2; y2; z2]))
        follow_setpoint = [x1; y1; z1];
    else
        follow_setpoint = [x2; y2; z2];
    end
    
end