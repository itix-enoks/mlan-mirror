%% System-Level Fairness & Spectral Efficiency: 802.11ac vs 802.11ax
%  Same 4-user heterogeneous scenario as multiuser_system.m / delay_drop.m
%  Metrics:
%    - Jain's Fairness Index  (network-level + per-user contribution)
%    - Spectral Efficiency    (network-level bps/Hz + per-user bps/Hz)
%
%  Spectral efficiency = throughput [bps] / channel bandwidth [Hz]
%  Bandwidth per user:
%    802.11ac TDMA : full CBW80 = 80 MHz (one user at a time)
%    802.11ax OFDMA: CBW80 / N_USERS = 20 MHz per user (sub-band RU)
%
%  Jain's Fairness Index:
%    JFI = ( sum(x_i) )^2  /  ( N * sum(x_i^2) )   in [1/N, 1]
%    Computed on per-user throughput so it captures scheduling fairness.
%clear; clc; close all;
%% =========================================================================
%% 1. TRAFFIC PROFILES  (identical to multiuser_system.m)
% =========================================================================
%% 1. TRAFFIC PROFILES - SET A
N_USERS = 4;

% User 1: Large Video
TRAFFIC(1).name = 'Video'; TRAFFIC(1).payload = 1500; TRAFFIC(1).mcs_ac = 8; TRAFFIC(1).mcs_ax = 10; TRAFFIC(1).snr = 30;

% User 2: Small Voice
TRAFFIC(2).name = 'Voice'; TRAFFIC(2).payload = 128;  TRAFFIC(2).mcs_ac = 4; TRAFFIC(2).mcs_ax = 5;  TRAFFIC(2).snr = 25;

% User 3: Large Video
TRAFFIC(3).name = 'Video'; TRAFFIC(3).payload = 1500; TRAFFIC(3).mcs_ac = 8; TRAFFIC(3).mcs_ax = 10; TRAFFIC(3).snr = 30;

% User 4: Small Voice
TRAFFIC(4).name = 'Voice'; TRAFFIC(4).payload = 128;  TRAFFIC(4).mcs_ac = 4; TRAFFIC(4).mcs_ax = 5;  TRAFFIC(4).snr = 25;
%% =========================================================================
%% 2. SIMULATION PARAMETERS
% =========================================================================
chan_coding   = 'LDPC';
delay_profile = 'Model-B';
cbw           = 'CBW80';
BW_HZ         = 80e6;          % 80 MHz channel bandwidth
numPackets    = 50;
N_USERS       = 4;
distances     = [3, 5, 7, 9];  % m
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
% ---- Per-user channel objects --------------------------------------------
tgax_usr = cell(1, N_USERS);
for u = 1:N_USERS
    tgax_usr{u} = wlanTGaxChannel( ...
        "ChannelBandwidth",        cbw,           ...
        "DelayProfile",            delay_profile,  ...
        "TransmitReceiveDistance", distances(u),   ...
        "LargeScaleFadingEffect",  'None');
end
%% =========================================================================
%% 3. 802.11ac — Round-Robin TDMA
% =========================================================================
fprintf('\n--- 802.11ac simulation ---\n');
bits_ac     = zeros(1, N_USERS);
slot_dur_ac = zeros(1, N_USERS);
for u = 1:N_USERS
    cfg = wlanVHTConfig( ...
        "ChannelBandwidth", cbw,             ...
        "MCS",              users(u).mcs_ac, ...
        "ChannelCoding",    chan_coding,      ...
        "APEPLength",       users(u).payload);
    fs       = wlanSampleRate(cfg);
    noiseVar = snrToNoiseVar(users(u).snr);
    reset(tgax_usr{u});
    ref_tx         = wlanWaveformGenerator(randi([0 1], cfg.PSDULength*8, 1), cfg);
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
    fprintf('  User %d (%s, %ddB): %d/%d pkts OK\n', ...
        u, users(u).type, users(u).snr, ...
        round(bits_ac(u)/(cfg.PSDULength*8)), numPackets);
end
% Per-user throughput [bps] — TDMA: each user owns 1/N of channel time
tp_ac_user  = bits_ac ./ (numPackets * slot_dur_ac * N_USERS);
tp_ac_total = sum(bits_ac) / (numPackets * sum(slot_dur_ac));
%% =========================================================================
%% 4. 802.11ax — HE-MU per user, OFDMA time model
% =========================================================================
fprintf('\n--- 802.11ax simulation ---\n');
cfg_ax_users = cell(1, N_USERS);
slot_dur_ax  = zeros(1, N_USERS);
bits_ax      = zeros(1, N_USERS);
psdu_ax      = zeros(1, N_USERS);
for u = 1:N_USERS
    cfg_ax_users{u}                        = wlanHEMUConfig(112);
    cfg_ax_users{u}.NumTransmitAntennas    = 1;
    cfg_ax_users{u}.User{1}.MCS           = users(u).mcs_ax;
    cfg_ax_users{u}.User{1}.ChannelCoding = chan_coding;
    cfg_ax_users{u}.User{1}.APEPLength    = users(u).payload;
    tmp_psdu = cfg_ax_users{u}.getPSDULength; psdu_ax(u) = tmp_psdu(1);
    fs_u = wlanSampleRate(cfg_ax_users{u});
    ref  = wlanWaveformGenerator( ...
               randi([0 1], psdu_ax(u)*8, 1), cfg_ax_users{u});
    slot_dur_ax(u) = size(ref, 1) / fs_u;
end
ofdma_slot = max(slot_dur_ax);
for pkt = 1:numPackets
    for u = 1:N_USERS
        cfg_u    = cfg_ax_users{u};
        noiseVar = snrToNoiseVar(users(u).snr);
        payload = randi([0 1], psdu_ax(u) * 8, 1);
        tx      = wlanWaveformGenerator(payload, cfg_u);
        reset(tgax_usr{u});
        rx = awgn(tgax_usr{u}(tx), users(u).snr, 'measured');
        ind       = wlanFieldIndices(cfg_u);
        info      = wlanHEOFDMInfo('HE-Data', cfg_u, 1);
        rxLTF     = rx(ind.HELTF(1):ind.HELTF(2), :);
        demodLTF  = wlanHEDemodulate(rxLTF, 'HE-LTF', cfg_u, 1);
        chanEst   = wlanHELTFChannelEstimate(demodLTF, cfg_u, 1);
        rxData    = rx(ind.HEData(1):ind.HEData(2), :);
        demodData = wlanHEDemodulate(rxData, 'HE-Data', cfg_u, 1);
        chanEstData  = chanEst(info.DataIndices, :, :);
        [eqSym, csi] = wlanHEEqualize( ...
            demodData(info.DataIndices,:,:), chanEstData, noiseVar, cfg_u, 'HE-Data', 1);
        rxBits = wlanHEDataBitRecover(eqSym, noiseVar, csi, cfg_u, 1);
        if ~any(payload ~= rxBits)
            bits_ax(u) = bits_ax(u) + psdu_ax(u) * 8;
        end
    end
end
for u = 1:N_USERS
    fprintf('  User %d (%s, %ddB): %d/%d pkts OK\n', ...
        u, users(u).type, users(u).snr, ...
        round(bits_ax(u)/(psdu_ax(u)*8)), numPackets);
end
total_time_ax = numPackets * ofdma_slot;
tp_ax_user    = bits_ax / total_time_ax;
tp_ax_total   = sum(bits_ax) / total_time_ax;
%% =========================================================================
%% 5. JAIN'S FAIRNESS INDEX
% =========================================================================
jfi_ac = jainFairness(tp_ac_user);
jfi_ax = jainFairness(tp_ax_user);
% Normalised throughput share per user (fair share = 1.0)
norm_share_ac = tp_ac_user / mean(tp_ac_user);
norm_share_ax = tp_ax_user / mean(tp_ax_user);
fprintf('\nJain''s Fairness Index:\n');
fprintf('  802.11ac: %.4f\n', jfi_ac);
fprintf('  802.11ax: %.4f\n', jfi_ax);
%% =========================================================================
%% 6. SPECTRAL EFFICIENCY
% =========================================================================
se_ac_net  = tp_ac_total / BW_HZ;                      % bps/Hz network
se_ax_net  = tp_ax_total / BW_HZ;
se_ac_user = tp_ac_user  / BW_HZ;                      % full band, time-shared
se_ax_user = tp_ax_user  / (BW_HZ / N_USERS);          % sub-band, always on
fprintf('\nSpectral Efficiency:\n');
fprintf('  802.11ac network: %.4f bps/Hz\n', se_ac_net);
fprintf('  802.11ax network: %.4f bps/Hz\n', se_ax_net);
%% =========================================================================
%% 7. PLOTS (Individual Files)
%  Replaces subplots with individual figures for separate saving.
%% =========================================================================
colors_ac = [0.85 0.25 0.25];
colors_ax = [0.20 0.45 0.85];
type_count_lg = 0; type_count_sm = 0;
xlabels = cell(1, N_USERS);
for u = 1:N_USERS
    if strcmp(users(u).type, 'Video')
        type_count_lg = type_count_lg + 1;
        xlabels{u} = sprintf('Large-Pkt %d', type_count_lg);
    else
        type_count_sm = type_count_sm + 1;
        xlabels{u} = sprintf('Small-Pkt %d', type_count_sm);
    end
end

% --- Plot 1: Network-level JFI ---
figure('Name','Network_JFI', 'Position',[100 100 600 500]);
jfi_vals = [jfi_ac, jfi_ax];
b1 = bar([1 2], jfi_vals, 0.45);
b1.FaceColor = 'flat';
b1.CData(1,:) = colors_ac;
b1.CData(2,:) = colors_ax;
set(gca, 'XTick',[1 2], 'XTickLabel',{'802.11ac','802.11ax'}, 'FontSize',11);
ylabel('Jain''s Fairness Index');
title('Network Jain''s Fairness Index');
ylim([0, 1.2]); grid on; box on;
for k = 1:2
    text(k, jfi_vals(k) + 0.04, sprintf('%.3f', jfi_vals(k)), ...
        'HorizontalAlignment','center','FontSize',12,'FontWeight','bold');
end
saveas(gcf, 'figures/jain_fairness_index.png');

% --- Plot 2: Per-user normalised throughput share ---
figure('Name','User_Fairness', 'Position',[150 150 800 500]);
norm_vals = [norm_share_ac(:), norm_share_ax(:)];
b2 = bar(1:N_USERS, norm_vals, 0.72);
b2(1).FaceColor = colors_ac;
b2(2).FaceColor = colors_ax;
set(gca,'XTick',1:N_USERS,'XTickLabel',xlabels,'FontSize',10);
ylabel('Normalised Throughput Share');
title('Per-User Fairness Contribution');
legend('802.11ac','802.11ax','Location','northeast');
grid on; box on;
barAnnotate(norm_vals, N_USERS, 2);
saveas(gcf, 'figures/user_fairness_contribution.png');

% --- Plot 3: Network Spectral Efficiency ---
figure('Name','Network_SE', 'Position',[200 200 600 500]);
se_net_vals = [se_ac_net, se_ax_net];
b3 = bar([1 2], se_net_vals, 0.45);
b3.FaceColor = 'flat';
b3.CData(1,:) = colors_ac;
b3.CData(2,:) = colors_ax;
set(gca,'XTick',[1 2],'XTickLabel',{'802.11ac','802.11ax'},'FontSize',11);
ylabel('Spectral Efficiency (bps/Hz)');
title('Network Spectral Efficiency');
ylim([0, max(se_net_vals)*1.35]);
grid on; box on;
for k = 1:2
    text(k, se_net_vals(k)+max(se_net_vals)*0.05, ...
        sprintf('%.4f', se_net_vals(k)), ...
        'HorizontalAlignment','center','FontSize',11,'FontWeight','bold');
end
saveas(gcf, 'figures/network_spectral_efficiency.png');

% --- Plot 4: Per-user Spectral Efficiency ---
figure('Name','User_SE', 'Position',[250 250 800 500]);
se_user_vals = [se_ac_user(:), se_ax_user(:)];
b4 = bar(1:N_USERS, se_user_vals, 0.72);
b4(1).FaceColor = colors_ac;
b4(2).FaceColor = colors_ax;
set(gca,'XTick',1:N_USERS,'XTickLabel',xlabels,'FontSize',10);
ylabel('SE (bps/Hz)');
title({'Per-User Spectral Efficiency'; 'ac: 80MHz shared | ax: 20MHz dedicated'});
legend('802.11ac (TDMA)','802.11ax (OFDMA)','Location','northeast');
grid on; box on;
barAnnotate(se_user_vals, N_USERS, 2);
saveas(gcf, 'figures/user_spectral_efficiency.png');
%% =========================================================================
%% LOCAL FUNCTIONS
% =========================================================================
function jfi = jainFairness(x)
    x = x(:);
    if all(x == 0), jfi = 0; return; end
    jfi = sum(x)^2 / (numel(x) * sum(x.^2));
end
function barAnnotate(vals, ngroups, nbars)
    groupwidth = min(0.8, nbars/(nbars+1.5));
    for s = 1:nbars
        xpos = (1:ngroups) - groupwidth/2 + (s-0.5)*groupwidth/nbars;
        for u = 1:ngroups
            v = vals(u, s);
            text(xpos(u), v + max(vals(:))*0.03, sprintf('%.3f', v), ...
                'HorizontalAlignment','center','FontSize',7.5);
        end
    end
end
function nv = snrToNoiseVar(snr_db)
    nv = 10^(-snr_db / 10);
end