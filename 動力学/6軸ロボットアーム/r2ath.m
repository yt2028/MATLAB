function [a,th]=r2ath(R)
th=0;dR=eye(3)-R;
if(sum(dR(:).^2)<eps) %Rが単位行列の場合
    a=[0;0;0]; 
    return;
end 
th=acos((trace(R)-1)/2);
a=0.5/sin(th)*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
if(nargout==1)
    a=a*th;
end
