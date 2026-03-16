%% Throughput vs Number of Users: 802.11ac vs 802.11ax
% ac: simulated as N parallel SU-VHT links (MU-MIMO approximation)
% ax: OFDMA with N users sharing the channel simultaneously via RU allocation
% wlanTGaxChannel only supports 1 TX antenna, so ac MU is modelled as
% N independent SU transmissions sharing the same time slot.

%% Initial parameters
cbw           = {'CBW20','CBW40','CBW80','CBW160'};
chan_cod       = {'BCC','LDPC'};
del_prof      = {'Model-A','Model-B','Model-C','Model-D','Model-E','Model-F'};
tx_rx_dist    = {1, 3, 5, 7, 10, 15};
trans_dir     = {'Downlink','Uplink'};
lscale_fading = {'None','Pathloss','Shadowing','Pathloss and shadowing'};

%% Fixed parameters
chan_bandwidth             = cbw{2};
chan_coding                = chan_cod{2};
delay_profile              = del_prof{1};
tx_rx_distance             = tx_rx_dist{1};
transmission_dir           = trans_dir{1};
large_scale_fading_effects = lscale_fading{1};
fixed_snr                  = 25;
fixed_mcs_ac               = 4;
fixed_mcs_ax               = 4;
fixed_payload              = 1024;
num_users_list             = [1, 2, 3, 4];

% AllocationIndex for wlanHEMUConfig — CBW40 needs 2-element vector
% 192 = one 242-tone RU per 20MHz subchannel, 1 user per RU
% 112 = two 106-tone RUs per 20MHz subchannel, 1 user per RU
% [192 192] -> 2 users, [112 192] -> 3 users, [112 112] -> 4 users
% For 1 user we use [192 192] and mark user 2 inactive (STAID=2046)
alloc_map = {[192 192], [192 192], [112 192], [112 112]};

%% Channel (1 TX antenna — required by wlanTGaxChannel)
tgax = genTGax(chan_bandwidth, delay_profile, tx_rx_distance, ...
               transmission_dir, large_scale_fading_effects);

%% Sweep
tp_ac = zeros(1, length(num_users_list));
tp_ax = zeros(1, length(num_users_list));

for i = 1:length(num_users_list)
    n = num_users_list(i);

    %% --- 802.11ac: N parallel SU-VHT links ---
    % Each user gets a full SU-VHT config; aggregate throughput = sum over users
    % All users share the same time slot duration (worst-case packet time)
    cfg_ac_su = wlanVHTConfig( ...
        "ChannelBandwidth", chan_bandwidth, ...
        "NumUsers",         1,             ...
        "MCS",              fixed_mcs_ac,  ...
        "ChannelCoding",    chan_coding,    ...
        "APEPLength",       fixed_payload);
    tp_ac(i) = calcThroughputAC_SU(cfg_ac_su, n, fixed_snr, tgax);

    %% --- 802.11ax: OFDMA ---
    cfg_ax = wlanHEMUConfig(alloc_map{i}, 'NumTransmitAntennas', 1);
    numConfigUsers = length(cfg_ax.User);
    for u = 1:numConfigUsers
        if u <= n
            cfg_ax.User{u}.MCS                = fixed_mcs_ax;
            cfg_ax.User{u}.ChannelCoding      = chan_coding;
            cfg_ax.User{u}.APEPLength         = fixed_payload;
            cfg_ax.User{u}.NumSpaceTimeStreams = 1;
        else
            cfg_ax.User{u}.STAID = 2046;  % inactive
        end
    end
    tp_ax(i) = calcThroughputMU_AX(cfg_ax, n, fixed_snr, tgax);
end

%% Plot
figure;
plot(num_users_list, tp_ac/1e6, 'r-o', 'LineWidth', 2); hold on;
plot(num_users_list, tp_ax/1e6, 'b-s', 'LineWidth', 2);
grid on;
xlabel('Number of Users');
ylabel('Aggregate Throughput (Mbps)');
legend('802.11ac (SU links)', '802.11ax (OFDMA)');
title(sprintf('Throughput vs Users (MCS=%d, SNR=%ddB, %s)', ...
    fixed_mcs_ac, fixed_snr, chan_bandwidth));
xticks(num_users_list);

%% -----------------------------------------------------------------------
% ac: run N independent SU-VHT links, sum their goodbits, divide by shared duration
function tp = calcThroughputAC_SU(cfg, numUsers, snr_db, tgax)
    numPackets  = 50;
    noiseVarEst = 10^(-snr_db/10);
    goodBits    = 0;
    dur         = 0;

    fs = wlanSampleRate(cfg);

    for u = 1:numUsers
        % Each user gets its own independent channel pass
        reset(tgax);
        for packet = 1:numPackets
            payl = randi([0,1], cfg.PSDULength*8, 1);
            tx   = wlanWaveformGenerator(payl, cfg);
            ind  = wlanFieldIndices(cfg);

            if u == 1 && packet == 1
                dur = size(tx,1) / fs;
            end

            rx = awgn(tgax(tx), snr_db, 'measured');

            rec_ltf   = rx(ind.VHTLTF(1):ind.VHTLTF(2), :);
            sym       = wlanVHTLTFDemodulate(rec_ltf, cfg);
            chanEst   = wlanVHTLTFChannelEstimate(sym, cfg);
            rec_data  = rx(ind.VHTData(1):ind.VHTData(2), :);
            decodBits = wlanVHTDataRecover(rec_data, chanEst, noiseVarEst, cfg);

            if length(decodBits) == length(payl) && sum(payl ~= decodBits) == 0
                goodBits = goodBits + length(payl);
            end
        end
    end

    % All users transmit in the same time slot — divide by one slot duration
    tp = goodBits / (numPackets * dur);
end

%% -----------------------------------------------------------------------
function tp = calcThroughputMU_AX(cfg, numUsers, snr_db, tgax)
    numPackets  = 50;
    noiseVarEst = 10^(-snr_db/10);
    goodBits    = 0;
    dur         = 0;

    fs             = wlanSampleRate(cfg);
    numConfigUsers = length(cfg.User);

    for packet = 1:numPackets
        % Build payload cell — empty for inactive users
        paylFull = cell(1, numConfigUsers);
        payl     = cell(1, numUsers);
        for u = 1:numConfigUsers
            if u <= numUsers && cfg.User{u}.STAID ~= 2046
                payl{u}     = randi([0,1], cfg.User{u}.APEPLength*8, 1);
                paylFull{u} = payl{u};
            else
                paylFull{u} = [];
            end
        end

        tx  = wlanWaveformGenerator(paylFull, cfg);
        ind = wlanFieldIndices(cfg);

        if packet == 1
            dur = size(tx,1) / fs;
        end

        rx = awgn(tgax(tx), snr_db, 'measured');

        % Decode each active user
        for u = 1:numUsers
            if cfg.User{u}.STAID == 2046
                continue;
            end

            info      = wlanHEOFDMInfo('HE-Data', cfg, u);
            rxLTF     = rx(ind.HELTF(1):ind.HELTF(2), :);
            demodLTF  = wlanHEDemodulate(rxLTF, 'HE-LTF', cfg, u);
            chanEst   = wlanHELTFChannelEstimate(demodLTF, cfg, u);
            rxData    = rx(ind.HEData(1):ind.HEData(2), :);
            demodData = wlanHEDemodulate(rxData, 'HE-Data', cfg, u);

            chanEstData  = chanEst(info.DataIndices, :, :);
            [eqSym, csi] = wlanHEEqualize( ...
                               demodData(info.DataIndices,:,:), ...
                               chanEstData, noiseVarEst, cfg, 'HE-Data', u);
            decodBits    = wlanHEDataBitRecover(eqSym, noiseVarEst, csi, cfg, u);

            nBits = min(length(payl{u}), length(decodBits));
            if sum(payl{u}(1:nBits) ~= decodBits(1:nBits)) == 0
                goodBits = goodBits + length(payl{u});
            end
        end
    end

    tp = goodBits / (numPackets * dur);
end

%% -----------------------------------------------------------------------
function tgax = genTGax(cbw, delp, txrxdist, trans_dir, ls_fad_eff)
    tgax = wlanTGaxChannel( ...
        "ChannelBandwidth",        cbw,      ...
        "ChannelFiltering",        1,        ...
        "DelayProfile",            delp,     ...
        "TransmitReceiveDistance", txrxdist, ...
        "TransmissionDirection",   trans_dir,...
        "LargeScaleFadingEffect",  ls_fad_eff);
end