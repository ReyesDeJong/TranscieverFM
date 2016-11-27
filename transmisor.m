close all
clear all
clc
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

modulacion=B*cos(2*pi*fc*t+ (2*pi*fDev)* triangular);
modulacion2=B*cos(2*pi*fc*t+ (2*pi*fDev2) * triangular);
figure
subplot(2,1,1)
plot(t,modulacion)
hold on
plot(t,signal,'black')
title('Señal modulada para una desviación de 75 y 100 Hz respectivamente');
ylabel('Amplitud [V]');
xlabel('Tiempo [s]');
subplot(2,1,2)
plot(t,modulacion2)
hold on
plot(t,signal,'black')
ylabel('Amplitud [V]');
xlabel('Tiempo [s]');

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
% as=fmdemod(ruido2,fc,Fs,fDev);
% plot(t,as);

                                                        %% parte cuatro y medio
dev3=10000;
modulacion=B*cos(2*pi*fc*t+ (2*pi*fDev) * triangular);
ruido = awgn(modulacion,15);
ruido2 = awgn(modulacion,30);
figure
as=fmdemod(ruido,fc,Fs,dev3);
plot(t,as);
hold on
plot(t,signal,'r')



%% modulacion buena
                                                       %% parte dos

B=1.5;
fc=10000;
fDev=7500;
%señal modulada

modulacion=B*cos(2*pi*fc*t+ (2*pi*fDev)* triangular);

%furier de modulacion
fourierm=fft(modulacion);
P2m = abs(fourierm/L);
P1m = P2m(1:L/2+1);
P1m(2:end-1) = 2*P1m(2:end-1);

figure
subplot(2,1,1)
plot(t,modulacion)
hold on
plot(t,signal,'black')
title('Señal modulada con indice de modulación 0.75.');
ylabel('Amplitud [V]');
xlabel('Tiempo [s]');
subplot(2,1,2)
plot(f,P1m);
ylabel('Amplitud ');
xlabel('Frecuencia [Hz]');
figure
subplot(2,1,1)
ruido = awgn(modulacion,15);
plot(t,ruido);
hold on 
plot(t,signal,'r');
title('Señal modulada con indice de modulación 0.75 y 15 SNR.');
ylabel('Amplitud [V]');
xlabel('Tiempo [s]');
subplot(2,1,2)
as=fmdemod(ruido,fc,Fs,dev3);
plot(t,as);
hold on
plot(t,signal,'r')
title('Señal demodulada con indice de modulación 0.75 y 15 SNR.');
ylabel('Amplitud [V]');
xlabel('Tiempo [s]');

