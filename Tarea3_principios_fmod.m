%% Señal cuadrada
vpp=1.5;         % Voltage peak to peak
f=500;           % Signal frecuency
Fs=200*f;        % Sampling signal rate
t=[0:Fs-1]'/(Fs);  % Sampling times
A = 1.5/2;       % Amplitude
w = f*2*pi;      % Signal frecuency (rad/s)
rho = 0;         % Signal phase
sq = A*square(w*t+rho);  % Square signal
figure
plot(t(201:800)*1000,sq(201:800),'LineWidth',2)
xlabel('Tiempo [ms]')
ylabel('Amplitud [V]')
title('Señal cuadrada en el tiempo')

%% Transformada de Fourier
T = 1/Fs;             % Sampling period
L = length(t);        % Length of signal      
Y = fft(sq);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f_fourier = Fs*(0:(L/2))/L;
figure
subplot(2,1,1)
plot(f_fourier(1:5000),(P1(1:5000)))
title('FFt de señal cuadrada','LineWidth',2)
xlabel('f (Hz)')
ylabel('|F(f)|')
subplot(2,1,2)
plot(f_fourier(1:5000),db(P1(1:5000)))
title('FFt de señal cuadrada','LineWidth',2)
xlabel('f (Hz)')
ylabel('|F(f)| db')
%% Portadora
Ap=1.5;
fp=10000;
wp=2*pi*fp;
sp=Ap*sin(wp.*t);
figure
plot(t(1:80)*1000,sp(1:80))
title('señal portadora','LineWidth',2)
xlabel('tiempo [ms]')
ylabel('Amplitud [V]')

    
Yp = fft(sp);
P2p = abs(Yp/L);
P1p = P2p(1:L/2+1);
P1p(2:end-1) = 2*P1p(2:end-1);
f_fourier = Fs*(0:(L/2))/L;
figure
subplot(2,1,1)
plot(f_fourier,(P1p))
title('FFt de señal portadora','LineWidth',2)
xlabel('f (Hz)')
ylabel('|F(f)|')
subplot(2,1,2)
plot(f_fourier,db(P1p))
title('FFt de señal portadora','LineWidth',2)
xlabel('f (Hz)')
ylabel('|F(f)| db')

%% Modulacion
dev1=75;     %Frecuency deviation in modulated siganl
dev2=100;    %Frecuency deviation in modulated siganl
dev3=500
dev=6*dev3;  % El optimo es cuando dev=6*dev3=3000 Hz
m=dev/f;
smod=fmmod(sq,fp,Fs,dev); %Modulated both chanels
figure
subplot(2,1,1)
plot(t(201:800)*1000,smod(201:800))
str = sprintf('Señal modualada en el tiempo con m = %f ',m);
title(str,'LineWidth',2);
xlabel('tiempo [ms]')
ylabel('Amplitud [V]')
subplot(2,1,2)
plot(t(201:800)*1000,sq(201:800),'LineWidth',2)
xlabel('Tiempo [ms]')
ylabel('Amplitud [V]')
title('Señal cuadrada en el tiempo')

Ymod = fft(smod);
P2mod = abs(Ymod/L);
P1mod = P2mod(1:L/2+1);
P1mod(2:end-1) = 2*P1mod(2:end-1);
f_fourier = Fs*(0:(L/2))/L;
figure
subplot(2,1,1)
plot(f_fourier,(P1mod))
str = sprintf('FFt de señal modulada con m = %f ',m);
title(str,'LineWidth',2);
xlabel('f (Hz)')
ylabel('|F(f)|')
subplot(2,1,2)
plot(f_fourier,db(P1mod))
str = sprintf('FFt de señal modulada con m = %f ',m);
title(str,'LineWidth',2);
xlabel('f (Hz)')
ylabel('|F(f)| db')
%% Demodulacion
sdmod=fmdemod(smod,fp,Fs,dev); %Demodulated both chanels
figure
plot(t(201:800)*1000,sdmod(201:800))
str = sprintf('Señal demodualada con m = %f ',m);
title(str,'LineWidth',2);
xlabel('tiempo [ms]')
ylabel('Amplitud [V]')

%% Adicion de ruido a señal modulada

smod_noise_15=awgn(smod,15,'measured');
smod_noise_30=awgn(smod,30,'measured');
figure
subplot(3,1,1)
plot(t(201:800)*1000,smod(201:800),'r','LineWidth',1)
xlabel('tiempo [ms]')
ylabel('Amplitud [V]')
subplot(3,1,2)
plot(t(201:800)*1000,smod_noise_15(201:800),'k')
str = sprintf('señal modulada con ruido snr = 15 ',m);
title(str,'LineWidth',2);

xlabel('tiempo [ms]')
ylabel('Amplitud [V]')

subplot(3,1,3)
plot(t(201:800)*1000,smod_noise_30(201:800),'k')
str = sprintf('señal modulada con ruido snr = 30 ',m);
title(str,'LineWidth',2);
xlabel('tiempo [ms]')
ylabel('Amplitud [V]')


%% Demodulacion con ruido

sdmod_noise_15=fmdemod(smod_noise_15,fp,Fs,dev); %Demodulated both chanels
sdmod_noise_30=fmdemod(smod_noise_30,fp,Fs,dev);
figure
subplot(2,1,1)
plot(t(201:800)*1000,sdmod(201:800),'r','LineWidth',2)
hold on
plot(t(201:800)*1000,sdmod_noise_15(201:800),'k')
hold off
title('señal demodulada con ruido','LineWidth',2)
str = sprintf('señal demodulada con ruido snr = 15 y m = %f ',m);
title(str,'LineWidth',2);
xlabel('tiempo [ms]')
ylabel('Amplitud [V]')
subplot(2,1,2)
plot(t(201:800)*1000,sdmod(201:800),'r','LineWidth',2)
hold on
plot(t(201:800)*1000,sdmod_noise_30(201:800),'k')
hold off
str = sprintf('señal demodulada con ruido snr = 30 y m = %f',m);
title(str,'LineWidth',2);
xlabel('tiempo [ms]')
ylabel('Amplitud [V]')

      
furier_sdmod_15 = abs(fft(sdmod_noise_15/L));
furier_sdmod_30 = abs(fft(sdmod_noise_30/L));
P2 = abs(Y/L);
furier_sdmod_15 = furier_sdmod_15(1:L/2+1);
furier_sdmod_30 = furier_sdmod_30(1:L/2+1);
furier_sdmod_15(2:end-1) = 2*furier_sdmod_15(2:end-1);
furier_sdmod_30(2:end-1) = 2*furier_sdmod_30(2:end-1);
f_fourier = Fs*(0:(L/2))/L;

figure
subplot(2,1,1)
plot(f_fourier(1:5000),(furier_sdmod_15(1:5000)))
str = sprintf('FFt de señal demodulada con ruido snr = 15 y m = %f',m);
title(str,'LineWidth',2);
xlabel('f (Hz)')
ylabel('|F(f)|')
subplot(2,1,2)
plot(f_fourier(1:5000),(furier_sdmod_30(1:5000)))
str = sprintf('FFt de señal demodulada con ruido snr = 30 y m = %f',m);
title(str,'LineWidth',2);
xlabel('f (Hz)')
ylabel('|F(f)|')


figure
subplot(2,1,1)
plot(f_fourier(1:5000),db(furier_sdmod_15(1:5000)))
str = sprintf('FFt de señal demodulada con ruido snr = 15 y m = %f',m);
title(str,'LineWidth',2);
xlabel('f (Hz)')
ylabel('|F(f)|db')
subplot(2,1,2)
plot(f_fourier(1:5000),db(furier_sdmod_30(1:5000)))
str = sprintf('FFt de señal demodulada con ruido snr = 30 y m = %f',m);
title(str,'LineWidth',2);
xlabel('f (Hz)')
ylabel('|F(f)|db')

%% Filtros asa banda
f=500;           % Signal frecuency
Fs=200*f;
fe=Fs/2;
rp=3;
rs=40;
df=max(2.5*dev,3*f)
df=min(df,5200);
df=3.5*df;
df=5250;
fmin=(10000-df);
fmax=(10000+df);
fmins=(10000-1.9*df);
fmaxs=(10000+1.9*df);
wp=[fmin fmax]/fe;
ws=[fmins fmaxs]/fe;
[n,wn]=buttord(wp,ws,rp,rs);
[b,a]=butter(n,wn);
[H,F]=freqz(b,a,f_fourier,Fs);

figure
subplot(2,1,1)
plot(F,(abs(H)))
str= sprintf('Filtro pasa banda de [%f, %f]',fmin,fmax);
title(str)
xlabel('Hz')
ylabel('|H(z)|')
subplot(2,1,2)
plot(F,db(abs(H)))
xlabel('Hz')
ylabel('|H(z)|db')


%%Filtrado 
Y15=filter(b,a,smod_noise_15);
Y30=filter(b,a,smod_noise_30);


%% Demodulacion con ruido y filtrado


sdmod_f_noise_15=fmdemod(smod_noise_15,fp,Fs,dev); 
sdmod_f_noise_30=fmdemod(smod_noise_30,fp,Fs,dev);
sdmod_noise_15=fmdemod(Y15,fp,Fs,dev); 
sdmod_noise_30=fmdemod(Y30,fp,Fs,dev);

figure
subplot(2,1,1)
plot(t(201:800)*1000,sdmod(201:800),'r','LineWidth',1)
hold on
plot(t(201:800)*1000,sdmod_noise_15(201:800),'k')
hold off
str = sprintf('señal demodulada y filtrada con ruido snr = 15 y m = %f',m);
title(str,'LineWidth',2);
xlabel('tiempo [ms]')
ylabel('Amplitud [V]')
subplot(2,1,2)
plot(t(201:800)*1000,sdmod(201:800),'r','LineWidth',1)
hold on
plot(t(201:800)*1000,sdmod_noise_30(201:800),'k')
hold off
str = sprintf('señal demodulada y filtrada con ruido snr = 30 y m = %f',m);
title(str,'LineWidth',2);
xlabel('tiempo [ms]')
ylabel('Amplitud [V]')

      
furier_sdmod_15 = abs(fft(sdmod_noise_15/L));
furier_sdmod_30 = abs(fft(sdmod_noise_30/L));
P2 = abs(Y/L);
furier_sdmod_15 = furier_sdmod_15(1:L/2+1);
furier_sdmod_30 = furier_sdmod_30(1:L/2+1);
furier_sdmod_15(2:end-1) = 2*furier_sdmod_15(2:end-1);
furier_sdmod_30(2:end-1) = 2*furier_sdmod_30(2:end-1);
f_fourier = Fs*(0:(L/2))/L;

figure
subplot(2,1,1)
plot(f_fourier(1:5000),(furier_sdmod_15(1:5000)))
str = sprintf('FFT señal demodulada y filtrada con ruido snr = 15 y m = %f',m);
title(str,'LineWidth',2)
xlabel('f (Hz)')
ylabel('|F(f)|')
subplot(2,1,2)
plot(f_fourier(1:5000),(furier_sdmod_30(1:5000)))
str = sprintf('FFT señal demodulada y filtrada con ruido snr = 30 y m = %f',m);
title(str,'LineWidth',2)
xlabel('f (Hz)')
ylabel('|F(f)| db')

figure
subplot(2,1,1)
plot(f_fourier(1:5000),db(furier_sdmod_15(1:5000)))
str = sprintf('FFT señal demodulada y filtrada con ruido snr = 15 y m = %f',m);
title(str,'LineWidth',2)
xlabel('f (Hz)')
ylabel('|F(f)|')
subplot(2,1,2)
plot(f_fourier(1:5000),db(furier_sdmod_30(1:5000)))
str = sprintf('FFT señal demodulada y filtrada con ruido snr = 30 y m = %f',m);
title(str,'LineWidth',2)
xlabel('f (Hz)')
ylabel('|F(f)| db')




