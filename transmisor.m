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
modulacion=B*cos(2*pi*fc*t+ (2*pi*fDev)/max(max(signal)) * triangular);
modulacion2=B*cos(2*pi*fc*t+ (2*pi*fDev2)/max(max(signal)) * triangular);
modulaciont(t,modulacion,modulacion2);

%furier de modulacion
fourierm=fft(modulacion);
P2m = abs(fourierm/L);
P1m = P2m(1:L/2+1);
P1m(2:end-1) = 2*P1m(2:end-1);
fourierm2=fft(modulacion2);
P2m2 = abs(fourierm2/L);
P1m2 = P2m2(1:L/2+1);
P1m2(2:end-1) = 2*P1m2(2:end-1);
modulacionfplot(f,P1m,P1m2);
                                                        %% parte tres
ruido = awgn(modulacion,15);
ruido2 = awgn(modulacion,30);
modulacionruido(t,ruido,ruido2);

ruido3 = awgn(modulacion2,15);
ruido4 = awgn(modulacion2,30);
modulacionruido(t,ruido3,ruido4);

                                                        %% parte cuatro

yq = hilbert(ruido).*exp(-j*2*pi*fc*t);
z = (1/(2*pi*fDev))*[zeros(1,size(yq',2)); diff(unwrap(angle(yq')))*Fs];
figure();
plot(t,z);