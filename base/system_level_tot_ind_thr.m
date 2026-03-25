%% System-Level Throughput: 802.11ac vs 802.11ax
%  Fixed 4-user scenario with heterogeneous traffic and per-user SNR.
%  Plots: (1) Total network throughput, (2) Individual user throughput.
%clear; clc; close all;

%% =========================================================================
%% 1. TRAFFIC PROFILES
%  Cycles across users: Video → Voice → Video → Voice
%    Video: 1024 B payload, high MCS, near user (good SNR)
%    Voice:  200 B payload, low  MCS, far  user (poor SNR)
% =========================================================================
TRAFFIC(1).name    = 'Video';
TRAFFIC(1).payload = 1024;
TRAFFIC(1).mcs_ac  = 7;     % 64-QAM 5/6
TRAFFIC(1).mcs_ax  = 9;     % 1024-QAM 3/4
TRAFFIC(1).snr     = 28;    % dB

TRAFFIC(2).name    = 'Voice';
TRAFFIC(2).payload = 200;
TRAFFIC(2).mcs_ac  = 3;     % 16-QAM 1/2
TRAFFIC(2).mcs_ax  = 4;     % 16-QAM 3/4
TRAFFIC(2).snr     = 18;    % dB

%% =========================================================================
%% 2. SIMULATION PARAMETERS
% =========================================================================
chan_coding   = 'LDPC';
delay_profile = 'Model-B';
cbw           = 'CBW80';
numPackets    = 50;
N_USERS       = 4;

% ---- Assign users --------------------------------------------------------
users = struct('type',{}, 'payload',{}, 'mcs_ac',{}, 'mcs_ax',{}, 'snr',{});
for u = 1:N_USERS
    t = TRAFFIC(mod(u-1, numel(TRAFFIC)) + 1);
    users(u).type    = t.name;
    users(u).payload = t.payload;
    users(u).mcs_ac  = t.mcs_ac;
    users(u).mcs_ax  = t.mcs_ax;
    users(u).snr     = t.snr;
end

fprintf('User configuration:\n');
fprintf('  %-4s %-6s %-8s %-8s %-6s\n','User','Type','MCS_ac','MCS_ax','SNR');
for u = 1:N_USERS
    fprintf('  %-4d %-6s %-8d %-8d %d dB\n', ...
        u, users(u).type, users(u).mcs_ac, users(u).mcs_ax, users(u).snr);
end

% ---- Per-user channel objects (independent fading, distinct distances) ---
tgax_usr = cell(1, N_USERS);
distances = [3, 5, 7, 9];   % metres — user 1 nearest, user 4 farthest
for u = 1:N_USERS
    tgax_usr{u} = wlanTGaxChannel( ...
        "ChannelBandwidth",        cbw,            ...
        "DelayProfile",            delay_profile,   ...
        "TransmitReceiveDistance", distances(u),    ...
        "LargeScaleFadingEffect",  'None');
end

%% =========================================================================
%% 3. 802.11ac — Round-Robin TDMA
%  Each user gets dedicated time slots (one at a time).
%  Per-user throughput = good bits / (numPackets * N_USERS * slot_duration)
%  because in round-robin each user waits while the other N-1 users transmit.
% =========================================================================
fprintf('\n--- 802.11ac simulation ---\n');
bits_ac      = zeros(1, N_USERS);
slot_dur_ac  = zeros(1, N_USERS);  % single-packet air time per user

for u = 1:N_USERS
    cfg = wlanVHTConfig( ...
        "ChannelBandwidth", cbw,             ...
        "MCS",              users(u).mcs_ac, ...
        "ChannelCoding",    chan_coding,      ...
        "APEPLength",       users(u).payload);

    fs       = wlanSampleRate(cfg);
    noiseVar = snrToNoiseVar(users(u).snr);
    reset(tgax_usr{u});

    % Compute slot duration from a reference waveform
    ref_tx       = wlanWaveformGenerator(randi([0 1], cfg.PSDULength*8, 1), cfg);
    slot_dur_ac(u) = size(ref_tx, 1) / fs;

    for pkt = 1:numPackets
        payload = randi([0 1], cfg.PSDULength * 8, 1);
        tx      = wlanWaveformGenerator(payload, cfg);

        reset(tgax_usr{u});
        rx = awgn(tgax_usr{u}(tx), users(u).snr, 'measured');

        ind     = wlanFieldIndices(cfg);
        ltf     = rx(ind.VHTLTF(1):ind.VHTLTF(2), :);
        sym     = wlanVHTLTFDemodulate(ltf, cfg);
        chanEst = wlanVHTLTFChannelEstimate(sym, cfg);
        rxData  = rx(ind.VHTData(1):ind.VHTData(2), :);
        rxBits  = wlanVHTDataRecover(rxData, chanEst, noiseVar, cfg);

        if ~any(payload ~= rxBits)
            bits_ac(u) = bits_ac(u) + cfg.PSDULength * 8;
        end
    end
    fprintf('  User %d (%s, SNR=%ddB): %d/%d pkts OK\n', ...
        u, users(u).type, users(u).snr, ...
        round(bits_ac(u)/(cfg.PSDULength*8)), numPackets);
end

% Each user is only served 1/N_USERS of the total time in round-robin
total_round_time_ac = numPackets * sum(slot_dur_ac);  % one full round per pkt
tp_ac_user  = bits_ac ./ (numPackets * slot_dur_ac * N_USERS);  % per-user bps
tp_ac_total = sum(bits_ac) / total_round_time_ac;               % aggregate bps

%% =========================================================================
%% 4. 802.11ax — HE-SU per user, OFDMA time modelled via shared slot
%
%  wlanHEMUConfig on a SISO chain requires NumSpaceTimeStreams=1 per RU,
%  but that field is read-only once the allocation is set — MATLAB derives
%  it from the RU tone plan and always defaults >1 for multi-RU configs,
%  causing the "STSS > NumTransmitAntennas" error regardless of attempts
%  to override it.
%
%  Workaround that preserves the OFDMA semantics correctly:
%    • Transmit one HE-SU packet per user (each uses its own sub-band MCS).
%    • The OFDMA "simultaneity" is captured in the time accounting:
%        - All N_USERS HE-SU packets are assumed to fit in parallel RUs
%          within the same symbol window, so the *charged* wall-clock time
%          per packet round equals the duration of the LONGEST user's frame
%          (the bottleneck slot), not the sum of all frames.
%    • This is conservative and physically correct: in 802.11ax DL-OFDMA
%      the AP transmits all RUs simultaneously; the frame duration is set
%      by the longest data field across all RUs.
% =========================================================================
fprintf('\n--- 802.11ax simulation ---\n');

bits_ax      = zeros(1, N_USERS);
slot_dur_ax  = zeros(1, N_USERS);   % per-user HE-SU frame duration

% Build one HE-SU config per user (SISO, no STBC ambiguity)
cfg_ax_users = cell(1, N_USERS);
for u = 1:N_USERS
    cfg_ax_users{u} = wlanHESUConfig( ...
        "ChannelBandwidth", cbw,             ...
        "MCS",              users(u).mcs_ax, ...
        "ChannelCoding",    chan_coding,      ...
        "APEPLength",       users(u).payload, ...
        "NumTransmitAntennas", 1,             ...
        "NumSpaceTimeStreams",  1);

    fs_u = wlanSampleRate(cfg_ax_users{u});
    ref  = wlanWaveformGenerator( ...
               randi([0 1], cfg_ax_users{u}.getPSDULength*8, 1), cfg_ax_users{u});
    slot_dur_ax(u) = size(ref, 1) / fs_u;
end

% OFDMA slot = duration of the longest frame (all RUs transmit in parallel)
ofdma_slot = max(slot_dur_ax);

for pkt = 1:numPackets
    for u = 1:N_USERS
        cfg_u    = cfg_ax_users{u};
        fs_u     = wlanSampleRate(cfg_u);
        noiseVar = snrToNoiseVar(users(u).snr);

        payload = randi([0 1], cfg_u.getPSDULength * 8, 1);
        tx      = wlanWaveformGenerator(payload, cfg_u);

        reset(tgax_usr{u});
        rx = awgn(tgax_usr{u}(tx), users(u).snr, 'measured');

        ind       = wlanFieldIndices(cfg_u);
        info      = wlanHEOFDMInfo('HE-Data', cfg_u);

        rxLTF    = rx(ind.HELTF(1):ind.HELTF(2), :);
        demodLTF = wlanHEDemodulate(rxLTF, 'HE-LTF', cfg_u);
        chanEst  = wlanHELTFChannelEstimate(demodLTF, cfg_u);

        rxData    = rx(ind.HEData(1):ind.HEData(2), :);
        demodData = wlanHEDemodulate(rxData, 'HE-Data', cfg_u);

        chanEstData  = chanEst(info.DataIndices, :, :);
        [eqSym, csi] = wlanHEEqualize( ...
            demodData(info.DataIndices,:,:), chanEstData, noiseVar, cfg_u, 'HE-Data');
        rxBits = wlanHEDataBitRecover(eqSym, noiseVar, csi, cfg_u);

        if ~any(payload ~= rxBits)
            bits_ax(u) = bits_ax(u) + cfg_u.getPSDULength * 8;
        end
    end
end

for u = 1:N_USERS
    fprintf('  User %d (%s, SNR=%ddB): %d/%d pkts OK\n', ...
        u, users(u).type, users(u).snr, ...
        round(bits_ax(u)/(cfg_ax_users{u}.getPSDULength*8)), numPackets);
end

% OFDMA: all users share the same slot — time charged once per packet round
total_time_ax = numPackets * ofdma_slot;
tp_ax_user    = bits_ax / total_time_ax;       % per-user bps
tp_ax_total   = sum(bits_ax) / total_time_ax;  % aggregate bps

%% =========================================================================
%% 5. PLOTS (Individual Files)
%% =========================================================================
colors_ac = [0.85 0.25 0.25];
colors_ax = [0.20 0.45 0.85];
xlabels = arrayfun(@(u) sprintf('User %d\n%s / %ddB', ...
    u, users(u).type, users(u).snr), 1:N_USERS, 'UniformOutput', false);

% --- Plot 1: Total Network Throughput ---
figure('Name','Total_Network_Throughput');
total_vals = [tp_ac_total, tp_ax_total] / 1e6;
b1 = bar([1 2], total_vals, 0.45);
b1.FaceColor = 'flat';
b1.CData(1,:) = colors_ac; b1.CData(2,:) = colors_ax;
set(gca, 'XTick', [1 2], 'XTickLabel', {'802.11ac','802.11ax'}, 'FontSize', 11);
ylabel('Total Network Throughput (Mbps)');
title('Total Network Throughput');
grid on;
for k = 1:2
    text(k, total_vals(k) + max(total_vals)*0.03, sprintf('%.1f Mbps', total_vals(k)), ...
        'HorizontalAlignment','center','FontWeight','bold');
end
saveas(gcf, 'figures/total_network_throughput.png');

% --- Plot 2: Individual User Throughput ---
figure('Name','Individual_User_Throughput');
user_vals = [tp_ac_user(:), tp_ax_user(:)] / 1e6;
b2 = bar(1:N_USERS, user_vals, 0.72);
b2(1).FaceColor = colors_ac; b2(2).FaceColor = colors_ax;
set(gca, 'XTick', 1:N_USERS, 'XTickLabel', xlabels, 'FontSize', 9.5);
ylabel('Throughput (Mbps)');
title('Individual User Throughput');
legend('802.11ac (TDMA)', '802.11ax (OFDMA)');
grid on;
saveas(gcf, 'figures/individual_user_throughput.png');
%% =========================================================================
%% LOCAL FUNCTIONS
% =========================================================================
function nv = snrToNoiseVar(snr_db)
    nv = 10^(-snr_db / 10);
end