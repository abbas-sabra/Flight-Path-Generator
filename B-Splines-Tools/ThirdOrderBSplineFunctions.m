function N3=ThirdOrderBSplineFunctions(t,x,k,SampleSize,N1,N2,periodic)
m1=length(t); m2=length(x);
N3=zeros(m1-3,m2);
if (periodic==1)
    k1=1; k2 = m1-3;
else
    k1=k-2; k2 = m1-k;
end
i =k1;
j1 =(i-1)*SampleSize+1;
j2 =j1+3*SampleSize-1;
if (periodic==1)
    N3(i,j1:j2)=(x(j1:j2)-t(i))./(t(i+2)-t(i)).*N2(i,j1:j2)+(t(i+3)-x(j1:j2))./(t(i+3)-t(i+1)).*N2(i+1,j1:j2);
else
    N3(i,j1:j2)=(t(i+3)-x(j1:j2))./(t(i+3)-t(i+1)).*N2(i+1,j1:j2);
end
for i=k1+1:k2-1
    j1 =(i-1)*SampleSize+1;
    j2 =j1+3*SampleSize-1;
    N3(i,j1:j2)=(x(j1:j2)-t(i))./(t(i+2)-t(i)).*N2(i,j1:j2)+(t(i+3)-x(j1:j2))./(t(i+3)-t(i+1)).*N2(i+1,j1:j2);
end
i=k2;
j1 =(i-1)*SampleSize+1;
j2 =j1+3*SampleSize-1;
if (periodic==1)
    N3(i,j1:j2)=(x(j1:j2)-t(i))./(t(i+2)-t(i)).*N2(i,j1:j2)+(t(i+3)-x(j1:j2))./(t(i+3)-t(i+1)).*N2(i+1,j1:j2);
else
    N3(i,j1:j2)=(x(j1:j2)-t(i))./(t(i+2)-t(i)).*N2(i,j1:j2);
end


end