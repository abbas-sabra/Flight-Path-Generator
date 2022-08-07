function N2=SecondOrderBSplineFunctions(t,x,k,SampleSize,N1,periodic)
m1=length(t); m2=length(x);
N2=zeros(m1-2,m2);
if (periodic==1)
    k1=1; k2 = m1-2;
else
    k1=k-1; k2 = m1-k;
end
i =k1;
j1 =(i-1)*SampleSize+1;
j2 =j1+2*SampleSize-1;
if (periodic==1)
    N2(i,j1:j2)=(x(j1:j2)-t(i))./(t(i+1)-t(i)).*N1(i,j1:j2)+(t(i+2)-x(j1:j2))./(t(i+2)-t(i+1)).*N1(i+1,j1:j2);
else
    N2(i,j1:j2)=(t(i+2)-x(j1:j2))./(t(i+2)-t(i+1)).*N1(i+1,j1:j2);
end
for i=k1+1:k2-1
    j1 =(i-1)*SampleSize+1;
    j2 =j1+2*SampleSize-1;
    N2(i,j1:j2)=(x(j1:j2)-t(i))./(t(i+1)-t(i)).*N1(i,j1:j2)+(t(i+2)-x(j1:j2))./(t(i+2)-t(i+1)).*N1(i+1,j1:j2);
end
i=k2;
j1 =(i-1)*SampleSize+1;
j2 =j1+2*SampleSize-1;
if (periodic==1)
    N2(i,j1:j2)=(x(j1:j2)-t(i))./(t(i+1)-t(i)).*N1(i,j1:j2)+(t(i+2)-x(j1:j2))./(t(i+2)-t(i+1)).*N1(i+1,j1:j2);
else
    N2(i,j1:j2)=(x(j1:j2)-t(i))./(t(i+1)-t(i)).*N1(i,j1:j2);
end


end
    
    
    
    