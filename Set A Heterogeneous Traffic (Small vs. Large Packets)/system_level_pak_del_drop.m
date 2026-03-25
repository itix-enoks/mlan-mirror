%% System-Level Packet Delay & Drop Rate: 802.11ac vs 802.11ax
%  Same 4-user heterogeneous scenario as multiuser_system.m.
%  Metrics:
%    - Average packet delay        (total network)
%    - Per-user packet delay
%    - Packet drop rate            (total network)
%    - Per-user packet drop rate   + labelled drop reason per packet
%
%  Drop reasons tracked:
%    'BitError'   — decoded bits differ from transmitted (PHY failure)
%    'LTFFail'    — channel estimate is all-zero (LTF field severely corrupted)
%    'NoData'     — data field indices are empty (frame too short / malformed)
%
%  Delay model (PHY-layer, matching what the WLAN Toolbox can observe):
%    Packet delay = propagation delay + transmission (air) time
%    Propagation delay = distance / speed_of_light
%    Transmission time = waveform length / sample_rate
%    For 802.11ac TDMA: add queuing delay = (position_in_round - 1) × slot_dur
%    For 802.11ax OFDMA: no queuing — all users transmit in the same slot
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
numPackets    = 50;
N_USERS       = 4;
C             = 3e8;            % speed of light (m/s)
distances     = [3, 5, 7, 9];  % per-user AP–STA distance (m)
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
fprintf('  %-4s %-6s %-8s %-8s %-6s %-8s\n', ...
    'User','Type','MCS_ac','MCS_ax','SNR','Dist(m)');
for u = 1:N_USERS
    fprintf('  %-4d %-6s %-8d %-8d %-6d %-8d\n', ...
        u, users(u).type, users(u).mcs_ac, users(u).mcs_ax, ...
        users(u).snr, distances(u));
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
%% 3. RESULT STORAGE
%  Per-packet records: delay (s) for successfully received packets,
%  drop flag + reason string for dropped packets.
% =========================================================================
% Struct arrays: one entry per (user, packet)
rec_ac = initRecords(N_USERS, numPackets);
rec_ax = initRecords(N_USERS, numPackets);
%% =========================================================================
%% 4. 802.11ac — Round-Robin TDMA
%
%  Delay breakdown per packet for user u at round-robin position u:
%    queuing_delay  = (u-1) × slot_dur_ac(u)   [wait for preceding users]
%    tx_delay       = slot_dur_ac(u)            [own transmission time]
%    prop_delay     = distances(u) / C
%    total = queuing + tx + prop
%
%  Drop reason priority: NoData > LTFFail > BitError
% =========================================================================
fprintf('\n--- 802.11ac simulation ---\n');
slot_dur_ac = zeros(1, N_USERS);
for u = 1:N_USERS
    cfg = wlanVHTConfig( ...
        "ChannelBandwidth", cbw,             ...
        "MCS",              users(u).mcs_ac, ...
        "ChannelCoding",    chan_coding,      ...
        "APEPLength",       users(u).payload);
    fs        = wlanSampleRate(cfg);
    noiseVar  = snrToNoiseVar(users(u).snr);
    ref_tx         = wlanWaveformGenerator(randi([0 1], cfg.PSDULength*8, 1), cfg);
    slot_dur_ac(u) = size(ref_tx, 1) / fs;
    prop_delay    = distances(u) / C;
    queue_delay   = (u - 1) * slot_dur_ac(u);  % TDMA position overhead
    pkt_delay_base = queue_delay + slot_dur_ac(u) + prop_delay;
    reset(tgax_usr{u});
    for pkt = 1:numPackets
        payload = randi([0 1], cfg.PSDULength * 8, 1);
        tx      = wlanWaveformGenerator(payload, cfg);
        reset(tgax_usr{u});
        rx = awgn(tgax_usr{u}(tx), users(u).snr, 'measured');
        ind = wlanFieldIndices(cfg);
        % --- Check 1: data field present ----------------------------------
        if ind.VHTData(2) - ind.VHTData(1) < 1
            rec_ac(u).dropped(pkt)    = true;
            rec_ac(u).reason{pkt}     = 'NoData';
            continue;
        end
        % --- Check 2: LTF / channel estimate ------------------------------
        ltf     = rx(ind.VHTLTF(1):ind.VHTLTF(2), :);
        sym     = wlanVHTLTFDemodulate(ltf, cfg);
        chanEst = wlanVHTLTFChannelEstimate(sym, cfg);
        if all(chanEst(:) == 0)
            rec_ac(u).dropped(pkt)    = true;
            rec_ac(u).reason{pkt}     = 'LTFFail';
            continue;
        end
        % --- Check 3: bit error -------------------------------------------
        rxData  = rx(ind.VHTData(1):ind.VHTData(2), :);
        rxBits  = wlanVHTDataRecover(rxData, chanEst, noiseVar, cfg);
        if any(payload ~= rxBits)
            rec_ac(u).dropped(pkt)    = true;
            rec_ac(u).reason{pkt}     = 'BitError';
        else
            rec_ac(u).dropped(pkt)    = false;
            rec_ac(u).reason{pkt}     = '';
            rec_ac(u).delay(pkt)      = pkt_delay_base;
        end
    end
    n_drop = sum(rec_ac(u).dropped);
    fprintf('  User %d (%s, %ddB): %d dropped / %d  |  avg delay = %.2f us\n', ...
        u, users(u).type, users(u).snr, n_drop, numPackets, ...
        mean(rec_ac(u).delay(~rec_ac(u).dropped)) * 1e6);
end
%% =========================================================================
%% 5. 802.11ax — HE-MU per user, OFDMA time model
%
%  Delay breakdown:
%    tx_delay   = ofdma_slot  (all users share the same transmission window)
%    prop_delay = distances(u) / C
%    queuing    = 0            (OFDMA: no per-user queue — simultaneous)
%    total = ofdma_slot + prop_delay
% =========================================================================
fprintf('\n--- 802.11ax simulation ---\n');
cfg_ax_users = cell(1, N_USERS);
slot_dur_ax  = zeros(1, N_USERS);
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
ofdma_slot = max(slot_dur_ax);   % all RUs bound to the longest frame
for u = 1:N_USERS
    cfg_u    = cfg_ax_users{u};
    noiseVar = snrToNoiseVar(users(u).snr);
    prop_delay     = distances(u) / C;
    pkt_delay_base = ofdma_slot + prop_delay;  % no queuing in OFDMA
    reset(tgax_usr{u});
    for pkt = 1:numPackets
        payload = randi([0 1], psdu_ax(u) * 8, 1);
        tx      = wlanWaveformGenerator(payload, cfg_u);
        reset(tgax_usr{u});
        rx = awgn(tgax_usr{u}(tx), users(u).snr, 'measured');
        ind  = wlanFieldIndices(cfg_u);
        info = wlanHEOFDMInfo('HE-Data', cfg_u, 1);
        % --- Check 1: data indices present --------------------------------
        if isempty(info.DataIndices)
            rec_ax(u).dropped(pkt) = true;
            rec_ax(u).reason{pkt}  = 'NoData';
            continue;
        end
        % --- Check 2: LTF / channel estimate ------------------------------
        rxLTF    = rx(ind.HELTF(1):ind.HELTF(2), :);
        demodLTF = wlanHEDemodulate(rxLTF, 'HE-LTF', cfg_u, 1);
        chanEst  = wlanHELTFChannelEstimate(demodLTF, cfg_u, 1);
        if all(chanEst(:) == 0)
            rec_ax(u).dropped(pkt) = true;
            rec_ax(u).reason{pkt}  = 'LTFFail';
            continue;
        end
        % --- Check 3: bit error -------------------------------------------
        rxData    = rx(ind.HEData(1):ind.HEData(2), :);
        demodData = wlanHEDemodulate(rxData, 'HE-Data', cfg_u, 1);
        chanEstData  = chanEst(info.DataIndices, :, :);
        [eqSym, csi] = wlanHEEqualize( ...
            demodData(info.DataIndices,:,:), chanEstData, noiseVar, cfg_u, 'HE-Data', 1);
        rxBits = wlanHEDataBitRecover(eqSym, noiseVar, csi, cfg_u, 1);
        if any(payload ~= rxBits)
            rec_ax(u).dropped(pkt) = true;
            rec_ax(u).reason{pkt}  = 'BitError';
        else
            rec_ax(u).dropped(pkt) = false;
            rec_ax(u).reason{pkt}  = '';
            rec_ax(u).delay(pkt)   = pkt_delay_base;
        end
    end
    n_drop = sum(rec_ax(u).dropped);
    fprintf('  User %d (%s, %ddB): %d dropped / %d  |  avg delay = %.2f us\n', ...
        u, users(u).type, users(u).snr, n_drop, numPackets, ...
        mean(rec_ax(u).delay(~rec_ax(u).dropped)) * 1e6);
end
%% =========================================================================
%% 6. AGGREGATE METRICS
% =========================================================================
% --- Per-user averages ----------------------------------------------------
avg_delay_ac   = zeros(1, N_USERS);
avg_delay_ax   = zeros(1, N_USERS);
drop_rate_ac   = zeros(1, N_USERS);
drop_rate_ax   = zeros(1, N_USERS);
for u = 1:N_USERS
    ok_ac = ~rec_ac(u).dropped;
    ok_ax = ~rec_ax(u).dropped;
    avg_delay_ac(u) = mean(rec_ac(u).delay(ok_ac)) * 1e6;   % µs
    avg_delay_ax(u) = mean(rec_ax(u).delay(ok_ax)) * 1e6;
    drop_rate_ac(u) = sum(rec_ac(u).dropped) / numPackets * 100;  % %
    drop_rate_ax(u) = sum(rec_ax(u).dropped) / numPackets * 100;
end
% --- Network-level averages -----------------------------------------------
all_delays_ac  = [];
all_delays_ax  = [];
for u = 1:N_USERS
    all_delays_ac = [all_delays_ac, rec_ac(u).delay(~rec_ac(u).dropped)]; %#ok<AGROW>
    all_delays_ax = [all_delays_ax, rec_ax(u).delay(~rec_ax(u).dropped)]; %#ok<AGROW>
end
net_avg_delay_ac = mean(all_delays_ac) * 1e6;
net_avg_delay_ax = mean(all_delays_ax) * 1e6;
total_drops_ac   = sum(cellfun(@(r) sum(r.dropped), num2cell(rec_ac)));
total_drops_ax   = sum(cellfun(@(r) sum(r.dropped), num2cell(rec_ax)));
net_drop_rate_ac = total_drops_ac / (N_USERS * numPackets) * 100;
net_drop_rate_ax = total_drops_ax / (N_USERS * numPackets) * 100;
%% =========================================================================
%% 7. PLOTS (Individual Files)
%% =========================================================================
% Logic: Replace subplot(2,2,n) with figure() and save after each.
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

% 1. Average Network Delay
figure();
net_vals = [net_avg_delay_ac, net_avg_delay_ax];
b = bar([1 2], net_vals, 0.45);
b.FaceColor = 'flat'; b.CData(1,:) = colors_ac; b.CData(2,:) = colors_ax;
set(gca, 'XTick', [1 2], 'XTickLabel', {'802.11ac','802.11ax'}, 'FontSize', 11);
ylabel('Avg Delay (\mus)'); title('Average Network Packet Delay');
saveas(gcf, 'figures/net_delay.png');

% 2. Per-User Average Delay
figure();
user_delay = [avg_delay_ac(:), avg_delay_ax(:)];
b2 = bar(1:N_USERS, user_delay, 0.72);
set(gca, 'XTick', 1:N_USERS, 'XTickLabel', xlabels, 'FontSize', 10);
ylabel('Avg Delay (\mus)'); title('Per-User Average Packet Delay');
legend('802.11ac (TDMA)', '802.11ax (OFDMA)', 'Location', 'northeast');
saveas(gcf, 'figures/user_delay.png');

%% =========================================================================
%% LOCAL FUNCTIONS
% =========================================================================
function rec = initRecords(N_USERS, numPackets)
    rec = struct('delay',   {}, 'dropped', {}, 'reason',  {});
    for u = 1:N_USERS
        rec(u).delay   = zeros(1, numPackets);   % 0 for dropped packets
        rec(u).dropped = false(1, numPackets);
        rec(u).reason  = repmat({''}, 1, numPackets);
    end
end
function barAnnotate(vals, ngroups, nbars)
    groupwidth = min(0.8, nbars/(nbars+1.5));
    for s = 1:nbars
        xpos = (1:ngroups) - groupwidth/2 + (s-0.5)*groupwidth/nbars;
        for u = 1:ngroups
            v = vals(u, s);
            text(xpos(u), v + max(vals(:))*0.03, sprintf('%.1f', v), ...
                'HorizontalAlignment','center','FontSize',7.5);
        end
    end
end
function nv = snrToNoiseVar(snr_db)
    nv = 10^(-snr_db / 10);
end