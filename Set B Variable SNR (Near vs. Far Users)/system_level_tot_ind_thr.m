% system-level throughput: 802.11ac vs 802.11ax
%  fixed 4-user scenario with heterogeneous traffic and per-user snr
%  plots: (1) total network throughput, (2) individual user throughput

clear; clc; close all;

% 1. traffic profiles - set b
N_USERS = 4;

% user 1: very near (fastest)
TRAFFIC(1).name = 'Near'; TRAFFIC(1).payload = 1024; TRAFFIC(1).mcs_ac = 9; TRAFFIC(1).mcs_ax = 11; TRAFFIC(1).snr = 38;

% user 2: near (fast)
TRAFFIC(2).name = 'Near'; TRAFFIC(2).payload = 1024; TRAFFIC(2).mcs_ac = 9; TRAFFIC(2).mcs_ax = 11; TRAFFIC(2).snr = 34;

% user 3: far (slow)
TRAFFIC(3).name = 'Far'; TRAFFIC(3).payload = 512; TRAFFIC(3).mcs_ac = 2; TRAFFIC(3).mcs_ax = 3; TRAFFIC(3).snr = 12;

% user 4: very far (slowest)
TRAFFIC(4).name = 'Far'; TRAFFIC(4).payload = 512; TRAFFIC(4).mcs_ac = 1; TRAFFIC(4).mcs_ax = 2; TRAFFIC(4).snr = 8;

% 2. simulation parameters
chan_coding = 'LDPC';
delay_profile = 'Model-B';
cbw = 'CBW80';
numPackets = 50;
N_USERS = 4;

% assign users
users = struct('type', {}, 'payload', {}, 'mcs_ac', {}, 'mcs_ax', {}, 'snr', {});

for u = 1:N_USERS
    t = TRAFFIC(mod(u - 1, numel(TRAFFIC)) + 1);
    users(u).type = t.name;
    users(u).payload = t.payload;
    users(u).mcs_ac = t.mcs_ac;
    users(u).mcs_ax = t.mcs_ax;
    users(u).snr = t.snr;
end

fprintf('User configuration:\n');
fprintf('  %-4s %-6s %-8s %-8s %-6s\n', 'User', 'Type', 'MCS_ac', 'MCS_ax', 'SNR');

for u = 1:N_USERS
    fprintf('  %-4d %-6s %-8d %-8d %d dB\n', ...
        u, users(u).type, users(u).mcs_ac, users(u).mcs_ax, users(u).snr);
end

% per-user channel objects (independent fading, distinct distances)
tgax_usr = cell(1, N_USERS);
distances = [3, 5, 7, 9]; % metres - user 1 nearest, user 4 farthest

for u = 1:N_USERS
    tgax_usr{u} = wlanTGaxChannel( ...
        "ChannelBandwidth", cbw, ...
        "DelayProfile", delay_profile, ...
        "TransmitReceiveDistance", distances(u), ...
        "LargeScaleFadingEffect", 'None');
end

% 802.11ac - round-robin tdma
%  each user gets dedicated time slots (one at a time)
%  per-user throughput = good bits / (numpackets * n_users * slot_duration)
%  because in round-robin each user waits while the other n-1 users transmit
fprintf('\n--- 802.11ac simulation ---\n');
bits_ac = zeros(1, N_USERS);
slot_dur_ac = zeros(1, N_USERS); % single-packet air time per user

for u = 1:N_USERS
    cfg = wlanVHTConfig( ...
        "ChannelBandwidth", cbw, ...
        "MCS", users(u).mcs_ac, ...
        "ChannelCoding", chan_coding, ...
        "APEPLength", users(u).payload);

    fs = wlanSampleRate(cfg);
    noiseVar = snrToNoiseVar(users(u).snr);
    reset(tgax_usr{u});

    % compute slot duration from a reference waveform
    ref_tx = wlanWaveformGenerator(randi([0 1], cfg.PSDULength * 8, 1), cfg);
    slot_dur_ac(u) = size(ref_tx, 1) / fs;

    for pkt = 1:numPackets
        payload = randi([0 1], cfg.PSDULength * 8, 1);
        tx = wlanWaveformGenerator(payload, cfg);

        reset(tgax_usr{u});
        rx = awgn(tgax_usr{u}(tx), users(u).snr, 'measured');

        ind = wlanFieldIndices(cfg);
        ltf = rx(ind.VHTLTF(1):ind.VHTLTF(2), :);
        sym = wlanVHTLTFDemodulate(ltf, cfg);
        chanEst = wlanVHTLTFChannelEstimate(sym, cfg);
        rxData = rx(ind.VHTData(1):ind.VHTData(2), :);
        rxBits = wlanVHTDataRecover(rxData, chanEst, noiseVar, cfg);

        if ~any(payload ~= rxBits)
            bits_ac(u) = bits_ac(u) + cfg.PSDULength * 8;
        end

    end

    fprintf('  User %d (%s, SNR=%ddB): %d/%d pkts OK\n', ...
        u, users(u).type, users(u).snr, ...
        round(bits_ac(u) / (cfg.PSDULength * 8)), numPackets);
end

% each user is only served 1/n_users of the total time in round-robin
total_round_time_ac = numPackets * sum(slot_dur_ac); % one full round per pkt
tp_ac_user = bits_ac ./ (numPackets * slot_dur_ac * N_USERS); % per-user bps
tp_ac_total = sum(bits_ac) / total_round_time_ac; % aggregate bps

% 802.11ax - he-mu per user, ofdma time modelled via shared slot
%
%  uses wlanhemuconfig with allocation index 112 (996-tone ru, cbw80),
%  one hemu object per user (siso, single ru, full bandwidth)
%  the ofdma simultaneity is captured in the time accounting:
%     all n_users he-mu packets are assumed to fit in parallel rus
%      within the same symbol window, so the *charged* wall-clock time
%      per packet round equals the duration of the longest user's frame
%      (the bottleneck slot), not the sum of all frames
%     this is conservative and physically correct: in 802.11ax dl-ofdma
%      the ap transmits all rus simultaneously; the frame duration is set
%      by the longest data field across all rus
fprintf('\n--- 802.11ax simulation ---\n');

bits_ax = zeros(1, N_USERS);
slot_dur_ax = zeros(1, N_USERS); % per-user he-su frame duration

% build one he-mu config per user (siso, no stbc ambiguity)
cfg_ax_users = cell(1, N_USERS);
psdu_ax = zeros(1, N_USERS);

for u = 1:N_USERS
    cfg_ax_users{u} = wlanHEMUConfig(112);
    cfg_ax_users{u}.NumTransmitAntennas = 1;
    cfg_ax_users{u}.User{1}.MCS = users(u).mcs_ax;
    cfg_ax_users{u}.User{1}.ChannelCoding = chan_coding;
    cfg_ax_users{u}.User{1}.APEPLength = users(u).payload;
    tmp_psdu = cfg_ax_users{u}.getPSDULength; psdu_ax(u) = tmp_psdu(1);
    fs_u = wlanSampleRate(cfg_ax_users{u});
    ref = wlanWaveformGenerator( ...
        randi([0 1], psdu_ax(u) * 8, 1), cfg_ax_users{u});
    slot_dur_ax(u) = size(ref, 1) / fs_u;
end

% ofdma slot = duration of the longest frame (all rus transmit in parallel)
ofdma_slot = max(slot_dur_ax);

for pkt = 1:numPackets

    for u = 1:N_USERS
        cfg_u = cfg_ax_users{u};
        fs_u = wlanSampleRate(cfg_u);
        noiseVar = snrToNoiseVar(users(u).snr);

        payload = randi([0 1], psdu_ax(u) * 8, 1);
        tx = wlanWaveformGenerator(payload, cfg_u);

        reset(tgax_usr{u});
        rx = awgn(tgax_usr{u}(tx), users(u).snr, 'measured');

        ind = wlanFieldIndices(cfg_u);
        info = wlanHEOFDMInfo('HE-Data', cfg_u, 1);

        rxLTF = rx(ind.HELTF(1):ind.HELTF(2), :);
        demodLTF = wlanHEDemodulate(rxLTF, 'HE-LTF', cfg_u, 1);
        chanEst = wlanHELTFChannelEstimate(demodLTF, cfg_u, 1);

        rxData = rx(ind.HEData(1):ind.HEData(2), :);
        demodData = wlanHEDemodulate(rxData, 'HE-Data', cfg_u, 1);

        chanEstData = chanEst(info.DataIndices, :, :);
        [eqSym, csi] = wlanHEEqualize( ...
            demodData(info.DataIndices, :, :), chanEstData, noiseVar, cfg_u, 'HE-Data', 1);
        rxBits = wlanHEDataBitRecover(eqSym, noiseVar, csi, cfg_u, 1);

        if ~any(payload ~= rxBits)
            bits_ax(u) = bits_ax(u) + psdu_ax(u) * 8;
        end

    end

end

for u = 1:N_USERS
    fprintf('  User %d (%s, SNR=%ddB): %d/%d pkts OK\n', ...
        u, users(u).type, users(u).snr, ...
        round(bits_ax(u) / (psdu_ax(u) * 8)), numPackets);
end

% ofdma: all users share the same slot - time charged once per packet round
total_time_ax = numPackets * ofdma_slot;
tp_ax_user = bits_ax / total_time_ax; % per-user bps
tp_ax_total = sum(bits_ax) / total_time_ax; % aggregate bps

% plots
colors_ac = [0.85 0.25 0.25];
colors_ax = [0.20 0.45 0.85];
type_count_near = 0; type_count_far = 0;
xlabels = cell(1, N_USERS);

for u = 1:N_USERS

    if strcmp(users(u).type, 'Near')
        type_count_near = type_count_near + 1;
        xlabels{u} = sprintf('Near %d', type_count_near);
    else
        type_count_far = type_count_far + 1;
        xlabels{u} = sprintf('Far %d', type_count_far);
    end

end

% plot 1: total network throughput
figure('Name', 'Total_Network_Throughput');
total_vals = [tp_ac_total, tp_ax_total] / 1e6;
b1 = bar([1 2], total_vals, 0.45);
b1.FaceColor = 'flat';
b1.CData(1, :) = colors_ac; b1.CData(2, :) = colors_ax;
set(gca, 'XTick', [1 2], 'XTickLabel', {'802.11ac', '802.11ax'}, 'FontSize', 11);
ylabel('Total Network Throughput (Mbps)');
title('Total Network Throughput');
grid on;

for k = 1:2
    text(k, total_vals(k) + max(total_vals) * 0.03, sprintf('%.1f Mbps', total_vals(k)), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end

saveas(gcf, 'figures/total_network_throughput.png');

% plot 2: individual user throughput
figure('Name', 'Individual_User_Throughput');
user_vals = [tp_ac_user(:), tp_ax_user(:)] / 1e6;
b2 = bar(1:N_USERS, user_vals, 0.72);
b2(1).FaceColor = colors_ac; b2(2).FaceColor = colors_ax;
set(gca, 'XTick', 1:N_USERS, 'XTickLabel', xlabels, 'FontSize', 9.5);
ylabel('Throughput (Mbps)');
title('Individual User Throughput');
legend('802.11ac (TDMA)', '802.11ax (OFDMA)');
grid on;
saveas(gcf, 'figures/individual_user_throughput.png');

% local functions
function nv = snrToNoiseVar(snr_db)
    nv = 10 ^ (-snr_db / 10);
end
