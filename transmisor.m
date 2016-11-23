                                                       %% parte uno
%tiempo
A = 0.75;
fm = 500;
Fs = 100000;
T = 1/Fs;
L = 1000; 
t = (0:L-1)*T;  
signal = A*square(2*pi*fm*t);
%frecuencia
fourier=fft(signal);
P2 = abs(fourier/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
moduladoraplot(t,signal,f,P1);

                                                       %% parte dos
%portadora
B=1.5;
fc=10000;
fDev=75;
fDev2=100;
portadora=B*cos(2*pi*fc*t);
portadoraplot(t,portadora);

%señal integrada
triangular=cumsum(signal)/Fs;
integradaplot(t,triangular);

%señal modulada
modulacion=B*cos(2*pi*fc*t+ (2*pi*fDev) * triangular);
modulacion2=B*cos(2*pi*fc*t+ (2*pi*fDev2) * triangular);
figure()
subplot(2,1,1);
plot(t,modulacion);
subplot(2,1,2);
plot(t,modulacion2);

%furier de modulacion
fourierm=fft(modulacion);
P2m = abs(modulacion/L);
P1m = P2m(1:L/2+1);
P1m(2:end-1) = 2*P1m(2:end-1);
fourierm2=fft(modulacion2);
P2m2 = abs(modulacion2/L);
P1m2 = P2m2(1:L/2+1);
P1m2(2:end-1) = 2*P1m2(2:end-1);
figure()
subplot(2,1,1);
plot(f,P1m);
subplot(2,1,2);
plot(f,P1m2);

                                                        %% parte tres
ruido = awgn(modulacion,15);
ruido2 = awgn(modulacion,30);
figure();
subplot(2,1,1);
plot(t,ruido);
subplot(2,1,2);
plot(t,ruido2);
figure()
ruido3 = awgn(modulacion2,15);
ruido4 = awgn(modulacion2,30);
subplot(2,1,1);
plot(t,ruido3);
subplot(2,1,2);
plot(t,ruido4);

                                                        %% parte cuatro
len = size(ruido,1);
if(len==1)
    ruido = ruido(:);
end


t = (0:1/Fs:((size(ruido,1)-1)/Fs))';
t = t(:,ones(1,size(ruido,2)));

yq = hilbert(ruido).*exp(-j*2*pi*fc*t);
z = (1/(2*pi*fDev))*[zeros(1,size(yq,2)); diff(unwrap(angle(yq)))*Fs];

% --- restore the output signal to the original orientation --- %
if(len == 1)
    z = z';
end
figure();
plot(t,z);