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
clear; clc; close all;
%% =========================================================================
%% 1. TRAFFIC PROFILES  (identical to multiuser_system.m)
% =========================================================================
TRAFFIC(1).name    = 'Video';
TRAFFIC(1).payload = 1024;
TRAFFIC(1).mcs_ac  = 7;
TRAFFIC(1).mcs_ax  = 9;
TRAFFIC(1).snr     = 28;
TRAFFIC(2).name    = 'Voice';
TRAFFIC(2).payload = 200;
TRAFFIC(2).mcs_ac  = 3;
TRAFFIC(2).mcs_ax  = 4;
TRAFFIC(2).snr     = 18;
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
%% 4. 802.11ax — HE-SU per user, OFDMA time model
% =========================================================================
fprintf('\n--- 802.11ax simulation ---\n');
cfg_ax_users = cell(1, N_USERS);
slot_dur_ax  = zeros(1, N_USERS);
bits_ax      = zeros(1, N_USERS);
for u = 1:N_USERS
    cfg_ax_users{u} = wlanHESUConfig( ...
        "ChannelBandwidth",    cbw,             ...
        "MCS",                 users(u).mcs_ax, ...
        "ChannelCoding",       chan_coding,      ...
        "APEPLength",          users(u).payload, ...
        "NumTransmitAntennas", 1,               ...
        "NumSpaceTimeStreams",  1);
    fs_u = wlanSampleRate(cfg_ax_users{u});
    ref  = wlanWaveformGenerator( ...
               randi([0 1], cfg_ax_users{u}.getPSDULength*8, 1), cfg_ax_users{u});
    slot_dur_ax(u) = size(ref, 1) / fs_u;
end
ofdma_slot = max(slot_dur_ax);
for pkt = 1:numPackets
    for u = 1:N_USERS
        cfg_u    = cfg_ax_users{u};
        noiseVar = snrToNoiseVar(users(u).snr);
        payload = randi([0 1], cfg_u.getPSDULength * 8, 1);
        tx      = wlanWaveformGenerator(payload, cfg_u);
        reset(tgax_usr{u});
        rx = awgn(tgax_usr{u}(tx), users(u).snr, 'measured');
        ind       = wlanFieldIndices(cfg_u);
        info      = wlanHEOFDMInfo('HE-Data', cfg_u);
        rxLTF     = rx(ind.HELTF(1):ind.HELTF(2), :);
        demodLTF  = wlanHEDemodulate(rxLTF, 'HE-LTF', cfg_u);
        chanEst   = wlanHELTFChannelEstimate(demodLTF, cfg_u);
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
    fprintf('  User %d (%s, %ddB): %d/%d pkts OK\n', ...
        u, users(u).type, users(u).snr, ...
        round(bits_ax(u)/(cfg_ax_users{u}.getPSDULength*8)), numPackets);
end
total_time_ax = numPackets * ofdma_slot;
tp_ax_user    = bits_ax / total_time_ax;
tp_ax_total   = sum(bits_ax) / total_time_ax;
%% =========================================================================
%% 5. JAIN'S FAIRNESS INDEX
%
%  Network-level JFI computed on per-user throughput vector.
%  Also compute the "fairness contribution" of each user: how much closer
%  to or farther from 1.0 that user pushes the index relative to a
%  hypothetical equal allocation — expressed as the normalised throughput
%  share  x_i / mean(x)  (1.0 = perfectly fair share).
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
%
%  SE_network [bps/Hz] = total_throughput / BW_HZ
%
%  Per-user SE:
%    802.11ac : user occupies full BW_HZ but only 1/N of the time
%               → SE_u = tp_ac_user(u) / BW_HZ
%    802.11ax : user occupies BW_HZ/N_USERS sub-band simultaneously
%               → SE_u = tp_ax_user(u) / (BW_HZ / N_USERS)
%
%  This correctly reflects that 802.11ax allocates a narrower sub-band
%  per user but uses it 100% of the time, whereas 802.11ac uses the full
%  band but only 1/N of the time — the per-user SE comparison shows which
%  standard uses its allocated spectrum more efficiently per user.
% =========================================================================
se_ac_net  = tp_ac_total / BW_HZ;                      % bps/Hz network
se_ax_net  = tp_ax_total / BW_HZ;
se_ac_user = tp_ac_user  / BW_HZ;                      % full band, time-shared
se_ax_user = tp_ax_user  / (BW_HZ / N_USERS);          % sub-band, always on
fprintf('\nSpectral Efficiency:\n');
fprintf('  802.11ac network: %.4f bps/Hz\n', se_ac_net);
fprintf('  802.11ax network: %.4f bps/Hz\n', se_ax_net);
%% =========================================================================
%% 7. PLOTS
% =========================================================================
colors_ac = [0.85 0.25 0.25];
colors_ax = [0.20 0.45 0.85];
xlabels = arrayfun(@(u) sprintf('User %d\n%s/%ddB', ...
    u, users(u).type, users(u).snr), 1:N_USERS, 'UniformOutput', false);
figure('Name','Fairness & Spectral Efficiency', 'Position',[80 80 1000 700]);
%% --- Row 1, Col 1: Network-level JFI ------------------------------------
subplot(2, 2, 1);
jfi_vals = [jfi_ac, jfi_ax];
b1 = bar([1 2], jfi_vals, 0.45);
b1.FaceColor = 'flat';
b1.CData(1,:) = colors_ac;
b1.CData(2,:) = colors_ax;
yline(1.0, 'k--', 'Perfect Fairness (1.0)', ...
    'LabelHorizontalAlignment','left', 'FontSize', 8);
set(gca, 'XTick',[1 2], 'XTickLabel',{'802.11ac','802.11ax'}, 'FontSize',10);
ylabel('Jain''s Fairness Index');
title('Network Jain''s Fairness Index');
ylim([0, 1.25]);
grid on; box on;
for k = 1:2
    text(k, jfi_vals(k) + 0.04, sprintf('%.3f', jfi_vals(k)), ...
        'HorizontalAlignment','center','FontSize',11,'FontWeight','bold');
end
%% --- Row 1, Col 2: Per-user normalised throughput share -----------------
subplot(2, 2, 2);
norm_vals = [norm_share_ac(:), norm_share_ax(:)];
b2 = bar(1:N_USERS, norm_vals, 0.72);
b2(1).FaceColor = colors_ac;
b2(2).FaceColor = colors_ax;
yline(1.0, 'k--', 'Fair share', 'FontSize', 8);
set(gca,'XTick',1:N_USERS,'XTickLabel',xlabels,'FontSize',9);
ylabel('Normalised Throughput Share  (1 = fair)');
title('Per-User Fairness Contribution');
legend('802.11ac','802.11ax','Location','northeast');
grid on; box on;
barAnnotate(norm_vals, N_USERS, 2);
%% --- Row 2, Col 1: Network Spectral Efficiency ---------------------------
subplot(2, 2, 3);
se_net_vals = [se_ac_net, se_ax_net];
b3 = bar([1 2], se_net_vals, 0.45);
b3.FaceColor = 'flat';
b3.CData(1,:) = colors_ac;
b3.CData(2,:) = colors_ax;
set(gca,'XTick',[1 2],'XTickLabel',{'802.11ac','802.11ax'},'FontSize',10);
ylabel('Spectral Efficiency (bps/Hz)');
title('Network Spectral Efficiency');
ylim([0, max(se_net_vals)*1.35]);
grid on; box on;
for k = 1:2
    text(k, se_net_vals(k)+max(se_net_vals)*0.05, ...
        sprintf('%.4f', se_net_vals(k)), ...
        'HorizontalAlignment','center','FontSize',10,'FontWeight','bold');
end
%% --- Row 2, Col 2: Per-user Spectral Efficiency --------------------------
subplot(2, 2, 4);
se_user_vals = [se_ac_user(:), se_ax_user(:)];
b4 = bar(1:N_USERS, se_user_vals, 0.72);
b4(1).FaceColor = colors_ac;
b4(2).FaceColor = colors_ax;
set(gca,'XTick',1:N_USERS,'XTickLabel',xlabels,'FontSize',9);
ylabel('SE (bps/Hz)');
title({'Per-User Spectral Efficiency'; ...
       '{\itac: full BW time-shared  |  ax: sub-band always-on}'});
legend('802.11ac (80 MHz shared)','802.11ax (20 MHz dedicated)', ...
       'Location','northeast');
grid on; box on;
barAnnotate(se_user_vals, N_USERS, 2);
sgtitle(sprintf('Fairness & Spectral Efficiency  |  %s  |  CBW80  |  LDPC  |  %d Users', ...
    delay_profile, N_USERS), 'FontWeight','bold');
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