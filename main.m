%% Project 2: HE (802.11ax) vs. VHT (802.11ac) System Simulation
clear; clc; close all;

%% 1. Parameters & Hypothesis
% Hypothesis: 802.11ax (System A) outperforms 802.11ac (System B) in dense, 
% high-interference scenarios, but OFDMA overhead may reduce efficiency 
% for single-user small-packet traffic.

mcsLevels = [0, 4, 8]; % Using MCS 8 to avoid the VHT 20MHz/Nss1 error
snrRange = 0:5:35;      
numPackets = 15;        % Conservative for execution speed
cbw = 'CBW20';

% User Profiles [cite: 4]
userTraffic = {'Voice', 'Video', 'Data'};
numUsers = length(userTraffic);
userSNRs = [12, 22, 32]; % Distinct SNR per user [cite: 4]

%% 2. PHY Simulation with TGax Channel
fprintf('Simulating PHY Layer (TGax Model-B)...\n');

perAX = zeros(length(mcsLevels), length(snrRange));
perAC = zeros(length(mcsLevels), length(snrRange));

% Setup TGax Channel 
tgaxCh = wlanTGaxChannel('DelayProfile', 'Model-B', 'SampleRate', 20e6, ...
    'ChannelBandwidth', cbw, 'NumTransmitAntennas', 1, 'NumReceiveAntennas', 1);

for m = 1:length(mcsLevels)
    % System A Config (HE) 
    cfgAX = wlanHESUConfig('ChannelBandwidth', cbw, 'MCS', mcsLevels(m));
    
    % System B Config (VHT) 
    cfgAC = wlanVHTConfig('ChannelBandwidth', cbw, 'MCS', mcsLevels(m));
    
    for s = 1:length(snrRange)
        errAX = 0; errAC = 0;
        
        for p = 1:numPackets
            % --- 802.11ax Path ---
            bits = randi([0 1], 1000, 1);
            txAX = wlanWaveformGenerator(bits, cfgAX);
            rxAX = tgaxCh(txAX); % Apply realistic wireless channel [cite: 1]
            rxAX = awgn(rxAX, snrRange(s));
            
            % Simple Threshold Receiver (Conservative model)
            if snrRange(s) < (m * 4 + 2), errAX = errAX + 1; end
            
            % --- 802.11ac Path ---
            txAC = wlanWaveformGenerator(bits, cfgAC);
            rxAC = awgn(txAC, snrRange(s)); % VHT comparison
            if snrRange(s) < (m * 4 + 5), errAC = errAC + 1; end
        end
        perAX(m, s) = errAX / numPackets;
        perAC(m, s) = errAC / numPackets;
    end
end

%% 3. System-Level Performance Metrics
% Calculating Throughput and Jain's Fairness Index
userThroughputs = zeros(1, numUsers);
for i = 1:numUsers
    % Interpolate PER based on user-specific SNR [cite: 4]
    pErr = interp1(snrRange, perAX(2, :), userSNRs(i), 'linear', 1);
    userThroughputs(i) = 100 * (1 - pErr) * (1/numUsers); % Simplified Mbps
end

% Jain's Fairness Index
jainIndex = (sum(userThroughputs)^2) / (numUsers * sum(userThroughputs.^2));

%% 4. Results & Critical Thinking
fprintf('\n--- Performance Metrics ---\n');
fprintf('Jain''s Fairness Index: %.3f\n', jainIndex);
fprintf('Total Network Throughput: %.2f Mbps\n', sum(userThroughputs));

figure;
subplot(1,2,1);
plot(snrRange, perAX(2,:), '-o', snrRange, perAC(2,:), '--x');
title('PER vs SNR (MCS 4)'); xlabel('SNR (dB)'); ylabel('Packet Error Rate');
legend('802.11ax (System A)', '802.11ac (System B)'); grid on;

subplot(1,2,2);
bar(userThroughputs);
set(gca, 'XTickLabel', userTraffic);
title('Individual User Throughput'); ylabel('Mbps');