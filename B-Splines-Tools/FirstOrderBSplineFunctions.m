function N1=FirstOrderBSplineFunctions(t,x,k,SampleSize,periodic)
m1=length(t); m2=length(x);
N1=zeros(m1-1,m2);
if (periodic==1)
    k1=1; k2 = m1-1;
else
    k1=k; k2 = m1-k;
end

for i=k1:k2
    j1=(i-1)*SampleSize+1;
    j2=j1+SampleSize-1;
    N1(i,j1:j2)=1;
end
end


