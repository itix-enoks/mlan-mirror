%% Project 2: HE-MU (802.11ax OFDMA) vs. VHT (802.11ac Single-User)
clear; clc; close all;

%% 1. Configuration
cbw     = 'CBW20';
fs      = wlanSampleRate('CBW20');
psduLen = 1000;   % bytes per user

% ---- System A: 802.11ax HE-MU (OFDMA) ----
% Use allocation index '00001101' → 3 users in 20 MHz (3x26-tone RUs).
% User count is derived from how many User cells the object creates.
cfgMU    = wlanHEMUConfig('00001101');
numUsers = length(cfgMU.User);   % correct way to get user count
fprintf('HE-MU allocation ''00001101'' -> %d users\n', numUsers);

userSNRs = linspace(12, 32, numUsers);

for u = 1:numUsers
    cfgMU.User{u}.MCS        = 4;
    cfgMU.User{u}.APEPLength = psduLen;
end

% ---- System B: 802.11ac VHT Single-User ----
cfgAC = wlanVHTConfig('ChannelBandwidth', cbw, 'MCS', 4, 'APEPLength', psduLen);

%% 2. Waveform Generation
bits = cell(1, numUsers);
for u = 1:numUsers
    bits{u} = randi([0 1], psduLen * 8, 1);
end
txAX = wlanWaveformGenerator(bits, cfgMU);
txAC = wlanWaveformGenerator(bits{1}, cfgAC);

fprintf('HE-MU waveform : %d samples\n', length(txAX));
fprintf('VHT   waveform : %d samples\n', length(txAC));

%% 3. Channel Modelling (TGax Model-B)
tgaxCh = wlanTGaxChannel( ...
    'DelayProfile',            'Model-B', ...
    'SampleRate',              fs, ...
    'ChannelBandwidth',        cbw, ...
    'NumTransmitAntennas',     1, ...
    'NumReceiveAntennas',      1, ...
    'TransmitReceiveDistance', 10);

reset(tgaxCh);  rxAX = tgaxCh(txAX);
reset(tgaxCh);  rxAC = tgaxCh(txAC);

%% 4. PER-based Throughput
% Sigmoid PER model around MCS-4 threshold (~18 dB).
% Replace with measured PER curves from your PHY simulation.
mcs4_threshold = 18;   % dB
slotTime       = 1e-3; % 1 ms TXOP

userThroughputs_AX = zeros(1, numUsers);
userThroughputs_AC = zeros(1, numUsers);

fprintf('\n%-6s %-10s %-8s\n', 'User', 'SNR (dB)', 'PER');
for u = 1:numUsers
    snr_u  = userSNRs(u);
    per_u  = 1 / (1 + exp(snr_u - mcs4_threshold));
    tput_u = (psduLen * 8 * (1 - per_u)) / slotTime;
    userThroughputs_AX(u) = tput_u;
    userThroughputs_AC(u) = tput_u;
    fprintf('%-6d %-10.1f %-8.4f\n', u, snr_u, per_u);
end

% MAC scheduling: AX serves all users per slot; AC round-robins
userThroughputs_AC = userThroughputs_AC / numUsers;

%% 5. Metrics
totalAX = sum(userThroughputs_AX);
totalAC = sum(userThroughputs_AC);
jainAX  = sum(userThroughputs_AX)^2 / (numUsers * sum(userThroughputs_AX.^2));
jainAC  = sum(userThroughputs_AC)^2 / (numUsers * sum(userThroughputs_AC.^2));

fprintf('\n%-35s %12s %12s\n', 'Metric', 'AX (OFDMA)', 'AC (SU-RR)');
fprintf('%s\n', repmat('-',1,60));
fprintf('%-35s %12.2f %12.2f\n', 'Total throughput (Mbps)', totalAX/1e6, totalAC/1e6);
fprintf('%-35s %12.4f %12.4f\n', 'Jains Fairness Index',    jainAX,      jainAC);

%% 6. Plots
figure;
bar(1:numUsers, [userThroughputs_AX; userThroughputs_AC]'/1e6);
xlabel('User'); ylabel('Throughput (Mbps)');
title('Per-User Throughput: 802.11ax OFDMA vs 802.11ac RR');
legend('802.11ax (HE-MU)','802.11ac (VHT RR)'); grid on;

figure;
subplot(1,2,1);
bar([totalAX totalAC]/1e6);
set(gca,'XTickLabel',{'802.11ax','802.11ac'}); grid on;
ylabel('Mbps'); title('Total Throughput');
subplot(1,2,2);
bar([jainAX jainAC]);
set(gca,'XTickLabel',{'802.11ax','802.11ac'}); grid on;
ylim([0 1]); title('Jains Fairness Index');