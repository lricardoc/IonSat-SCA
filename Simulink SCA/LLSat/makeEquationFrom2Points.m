function [m, b0, eq] = makeEquationFrom2Points(x1,y1,x2,y2)
        syms x y;
        if x1==x2 && y1==y2
            disp('Need 2 distinct points');
            a=NaN;
            b=NaN;
            c=NaN;
            eq="null";
            return;
        else if x1==x2
            b=0;
            a=1;
            c=x1;
        else
            %for lines m not_equal to inf 
            %y=mx+c
            %m is coeff(1) and coeff(2) is c 
            coefficient=polyfit([x1 x2],[y1 y2],1);
            a=-coefficient(1);
            c=coefficient(2);
            b=1;
            m=-a/b;
            b0=c/b;
        end
        eq=y==c/b-a/b*x;
    end