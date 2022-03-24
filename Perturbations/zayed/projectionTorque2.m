function [T]=projectionTorque2(G,vorbit)
% les 4 points qui delimite le cadre 2
A=[0.17,0.11,0.05];%ok
B=[0.17,0.11,-0.05];
C=[-0.17,0.11,-0.05];
D=[-0.17,0.11,0.05];%ok


torque=[0,0,0];
%projection of B
pBz=0.05;
t=(pBz-B(3))/vorbit(3);
pBx=B(1)+t*vorbit(1);
pBy=B(2)+t*vorbit(2);

%projection of c
pCz=pBz;
pCx=C(1)+t*vorbit(1);%% 
pCy=pBy;%% because the projection is in this case //
%vecteur directeur de notre droite
ux=A(1)-pBx;
uy=A(2)-pBy;
uz=A(3)-pBz;

if pCy<0.28
    if pBx<0.17
        
        if pBx>-0.17 %%we caculate(frcement pCx<-0.17)
            %pas
            N=100;%%%%%%%%%%%%%%%%%%%can be changed
            pas=ux/N;
            %integrale de rieman
      
            for i=0:N-1
                x=pBx+i*pas;
                t=(x-A(1))/ux;
                y=A(2)+t*uy;
                %geometric center for dA
                Gc=[x+(pas/2),(0.28+y)/2,pBz];
                Rs=Gc-G;
                torque= torque +cross(Rs,vorbit)*((0.28-y)*pas);
            end
            %the other surface
            Gc=[(pBx-0.17)/2,(pBy+0.28)/2,pBz];
            Rs=Gc-G;
            torque=torque+(pBx+0.17)*(0.28-pBy)*cross(Rs,vorbit);
        else
            %%we calculate(la droite (ApB) coupe [segment]
            N=200;
            pas=0.34/N;
            
            for i=0:N-1
                x=-0.17+i*pas;
                t=(x-A(1))/ux;
                y=A(2)+t*uy;
                Gc=[x+(pas/2),(y+0.28)/2,pBz];
                Rs=Gc-G;
                torque= torque +cross(Rs,vorbit)*((0.28-y)*pas);
                
            end
            
            
            
        end
        
    elseif pCx<0.17 %%we konw already that pCx>-0.17
        %we calculate
        ux=D(1)-pCx;
        uy=D(2)-pCy;
        uz=D(3)-pCz;
        N=100;
        pas=ux/N;%negative always
       
        for i=0:N-1
            x=pCx+i*pas;
            t=(x-D(1))/ux;
            y=D(2)+t*uy;
            %geometric center for dA
            Gc=[x+(pas/2),(y+0.28)/2,pCz];
            Rs=Gc-G;
            torque= torque +cross(Rs,vorbit)*((0.28-y)*pas);
            
        end
        Gc=[(pCx+0.17)/2,(pCy+0.28)/2,pCz];
        Rs=Gc-G;
        torque=torque+(0.17-pCx)*(0.28-pCy)*cross(Rs,vorbit);
    else
        ux=D(1)-pCx;
        uy=D(2)-pCy;
        uz=D(3)-pCz;
        %we calculate(coupe droite segment)
        N=200;
        pas=-0.34/N;
     
        for i=0:N-1
            x=0.17+i*pas;
            t=(x-D(1))/ux;
            y=D(2)+t*uy;
            Gc=[x+(pas/2),(y+0.28)/2,pCz];
            Rs=Gc-G;
            torque= torque +cross(Rs,vorbit)*((0.28-y)*pas);
            
        end
        
    end
else
    %let Dprim Aprim to be like
    Dprim=[-0.17,0.28,0.05];
    Aprim=[0.17,0.28,0.05];
    if pBx<0.17
        ux=A(1)-pBx;
        uy=A(2)-pBy;
        uz=A(3)-pBz;
        %%%%
        ADpv=A-Dprim;
        %%%%%
        tprim=(pBy-A(2))/ADpv(2);
        %on compare
        xc=A(1)+tprim*ADpv(1);
        if pBx<xc%%%derriere suivant les x negative
            N=200;%%%%%%%%%%%%%%%%%%%can be changed
            pas=0.34/N;
            %integrale de rieman
            
            for i=0:N-1
                x=-0.17+i*pas;%%%intersection DDprim
                t=(x-A(1))/ux;
                y=A(2)+t*uy;
                %geometric center for dA
                Gc=[x+(pas/2),(y+0.28)/2,pBz];
                Rs=Gc-G;
                torque= torque +cross(Rs,vorbit)*((0.28-y)*pas);
            end
            
        else
            %we calculate%%%%%%%%%%problem(we can use diagonale)
            t=(0.28-A(2))/uy;%%intersection DprimAprim
            x=A(1)+t*ux;
            N=100;%%%%%%%%%%%%%%%%%%%can be changed
            pas=(0.17-x)/N;
            %integrale de rieman
          
            for i=0:N-1
                x=x+i*pas;
                t=(x-A(1))/ux;
                y=A(2)+t*uy;
                %geometric center for dA
                Gc=[x+(pas/2),(y+0.28)/2,pBz];
                Rs=Gc-G;
                torque= torque +cross(Rs,vorbit)*((0.28-y)*pas);
            end
            
            
            
        end
        %%elseif pCx<0.17
        %we calculate
    else
        ux=D(1)-pCx;
        uy=D(2)-pCy;
        uz=D(3)-pCz;
        %%%%
        DApv=D-Aprim;
        %%%%%
        tprim=(pCy-A(2))/DApv(2);
        %on compare
        xc=D(1)+tprim*DApv(1);
        
        if pCx>xc
            N=200;%%%%%%%%%%%%%%%%%%%can be changed
            pas=-0.34/N;
            %integrale de rieman
            
            for i=0:N-1
                x=0.17+i*pas;
                t=(x-D(1))/ux;
                y=D(2)+t*uy;
                %geometric center for dA
                Gc=[x+(pas/2),(y+0.28)/2,pCz];
                Rs=Gc-G;
                torque= torque +cross(Rs,vorbit)*((0.28-y)*pas);
            end
            
        else
            %we calculate%%%%%%%%%%problem(we can use diagonale)
            t=(0.28-D(2))/uy;
            x=D(1)+t*ux;
            N=100;%%%%%%%%%%%%%%%%%%%can be changed
            pas=(-0.17-x)/N;
            %integrale de rieman
           
            for i=0:N-1
                x=x+i*pas;
                t=(x-D(1))/ux;
                y=D(2)+t*uy;
                %geometric center for dA
                Gc=[x+(pas/2),(y+0.28)/2,pCz];
                Rs=Gc-G;
                torque= torque +cross(Rs,vorbit)*((0.28-y)*pas);
            end
            
            
            
        end
        
    end
    
end
T=torque;

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



