close all;
clear;
clc;

%% plant
param = plant_param();
sysc = plant_sysc(param);
ssc = ss(sysc.A,sysc.B,sysc.C,sysc.D);

%% built-in bode plot
[mag_bode,phase_bode,w_bode] = bode(ssc);
mag_bode = squeeze(mag_bode(1,1,:));
phase_bode = squeeze(phase_bode(1,1,:));

%% impulse bode plot
% sampling period (s)
dt = 0.001;

% time vector
t = (0:dt:10)';
t = t(1:end-1);

% bode frequency
N = length(t);
w_impulse = (2*pi*((1:N/2)-1)/(N*dt))';
w_impulse(1) = w_bode(1);

% impulse signal
u = zeros(size(t));
u(1) = 1/dt;

% time response
y = lsim(ssc,u,t);
y = y(:,1);

% frequency response
Puu = fft(u);
Pyy = fft(y);
Puy = (Pyy.*conj(Puu))./(Puu.*conj(Puu));

% transfer function
mag_impulse = abs(Puy);
mag_impulse = mag_impulse(1:N/2);

%% sine bode plot
% sampling period (s)
dt = 0.001;

% time vector
t = (0:dt:10)';
t = t(1:end-1);

% sine frequency
N = length(t);
w_sine = (2*pi*((1:N/2)-1)/(N*dt))';
w_sine(1) = w_bode(1);

% sine frequency
w_min = 2*pi/t(end);
w_max = 2*pi/(2*dt);
w_vec = logspace(floor(log10(w_min)),floor(log10(w_max)),11)';

mag_sine = [];
mag_sine_interp = [];
for i = 1:length(w_vec)
    w = w_vec(i);

    % sine signal
    u = sin(w*t);

    % time response
    y = lsim(ssc,u,t);
    y = y(:,1);

    % frequency response
    Puu = fft(u);
    Pyy = fft(y);
    Puy = (Pyy.*conj(Puu))./(Puu.*conj(Puu));

    % transfer function
    mag = abs(Puy);
    mag = mag(1:N/2);
    mag_sine = [mag_sine,mag];
    mag_sine_interp = [
        mag_sine_interp;
        [w,interp1(w_sine,mag,w)];
        ];
end

%% plotting
figure;
semilogx(w_bode,20*log10(mag_bode),"-r","LineWidth",1); hold on;
semilogx(w_impulse,20*log10(mag_impulse),"--b","LineWidth",1);
semilogx(w_sine,20*log10(mag_sine),'-.k');
scatter(mag_sine_interp(:,1),20*log10(mag_sine_interp(:,2)),"k");
legend(["built-in bode","impulse","sine"]);
ax = gca(); ax.FontSize = 12;
xlabel("frequency (rad/s)");
ylabel("magnitude (dB)");
