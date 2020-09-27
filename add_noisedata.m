function [signal,noise]=add_noisedata(s,data,fs,fs1,snr)
%原理就是将信号与噪声直接相加
s=s(:);
sL=length(s);
if fs~=fs1
    x=resample(data,fs,fs1);
else
    x=data;
end
x=x(:);
xL=length(x);
if xL>=sL
    x=x(1:sL);
else
    disp('warning')
    x=[x;zeros(sL-xL,1)];
 end
        Sr=snr;
        
        Es=sum(s.*s);
        Ev=sum(x.*x);
        a=sqrt(Es/Ev/(10^(Sr/10)));
        noise=a*x;
        signal=s+noise;
        end