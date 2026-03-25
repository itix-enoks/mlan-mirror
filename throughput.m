%% Throughput vs Guard Interval: 802.11ac vs 802.11ax

% Initial parameters
cbw = {'CBW20','CBW40','CBW80','CBW160'};
num_usr = 1:4;
mcs_ac = 0:9;
mcs_ax = 0:11;
chan_cod = {'BCC','LDPC'};
guard_i = {'short','long'};
guard_i_ax = {3.2, 1.6, 0.8};
payload_len = {1024, 200};
del_prof = {'Model-A','Model-B','Model-C','Model-D','Model-E','Model-F'};
tx_rx_dist = {1, 3, 5, 7, 10, 15};
trans_dir = {'Downlink','Uplink'};
lscale_fading = {'None','Pathloss','Shadowing','Pathloss and shadowing'};

%% Fixed parameters
chan_bandwidth             = cbw{3};           % CBW80 — wider band, more multipath sensitivity
chan_coding                = chan_cod{2};      % LDPC — keep robust coding
delay_profile              = del_prof{2};      % Model-B: slight delay spread, not deadly
tx_rx_distance             = tx_rx_dist{2};    % 3m — close enough to survive, enough multipath
large_scale_fading_effects = lscale_fading{1}; % None — isolate GI effect cleanly
fixed_snr                  = 30;               % high SNR so failures are GI-driven not noise-driven
fixed_mcs_ac               = 8;                % 256-QAM — very sensitive to ISI
fixed_mcs_ax               = 9;                % 1024-QAM equivalent sensitivity
fixed_payload              = payload_len{1};   % 1024 bytes

%% Setup channel
tgax = genTGax(chan_bandwidth, delay_profile, tx_rx_distance, ...
               trans_dir{1}, large_scale_fading_effects);

%% Sweep guard intervals
gi_ac_labels = {'0.4us', '0.8us'};
gi_ax_values = {0.8, 1.6, 3.2};
gi_ax_labels = {'0.8us', '1.6us', '3.2us'};

tp_ac = zeros(1, length(guard_i));
tp_ax = zeros(1, length(gi_ax_values));

% --- ac sweep ---
for i = 1:length(guard_i)
    cfg_ac = wlanVHTConfig( ...
        "ChannelBandwidth", chan_bandwidth, ...
        "NumUsers",         1,             ...
        "MCS",              fixed_mcs_ac,  ...
        "ChannelCoding",    chan_coding,    ...
        "GuardInterval",    guard_i{i},    ...
        "APEPLength",       fixed_payload);
    tp_ac(i) = calcThroughputSingle(cfg_ac, 'ac', fixed_snr, tgax);
end

% --- ax sweep ---
for i = 1:length(gi_ax_values)
    cfg_ax = wlanHESUConfig( ...
        "ChannelBandwidth", chan_bandwidth,  ...
        "MCS",              fixed_mcs_ax,   ...
        "ChannelCoding",    chan_coding,     ...
        "GuardInterval",    gi_ax_values{i},...
        "APEPLength",       fixed_payload);
    cfg_ax.HELTFType = gi_ax_values{i} / 0.8;
    tp_ax(i) = calcThroughputSingle(cfg_ax, 'ax', fixed_snr, tgax);
end

%% Plot
figure;
subplot(1,2,1);
bar(1:length(guard_i), tp_ac/1e6, 'r');
set(gca, 'XTick', 1:length(guard_i), 'XTickLabel', gi_ac_labels);
ylabel('Throughput (Mbps)');
title('802.11ac');
grid on;

subplot(1,2,2);
bar(1:length(gi_ax_values), tp_ax/1e6, 'b');
set(gca, 'XTick', 1:length(gi_ax_values), 'XTickLabel', gi_ax_labels);
ylabel('Throughput (Mbps)');
title('802.11ax');
grid on;

sgtitle(sprintf('Throughput vs Guard Interval (MCS=%d, SNR=%ddB, %s)', ...
    fixed_mcs_ac, fixed_snr, chan_bandwidth));

%% -----------------------------------------------------------------------
function tp = calcThroughputSingle(cfg, mode, snr_db, tgax)
    numPackets  = 50;
    noiseVarEst = 10^(-snr_db/10);
    goodBits    = 0;

    % Compute duration from actual waveform
    fs = wlanSampleRate(cfg);
    if strcmp(mode, 'ac')
        dummy = randi([0,1], cfg.PSDULength * 8, 1);
    else
        dummy = randi([0,1], cfg.getPSDULength * 8, 1);
    end
    refWave = wlanWaveformGenerator(dummy, cfg);
    dur     = size(refWave, 1) / fs;

    for packet = 1:numPackets
        if strcmp(mode, 'ac')
            payl      = randi([0,1], cfg.PSDULength * 8, 1);
            tx        = wlanWaveformGenerator(payl, cfg);
            ind       = wlanFieldIndices(cfg);
            rx        = awgn(tgax(tx), snr_db, 'measured');

            rec_ltf   = rx(ind.VHTLTF(1):ind.VHTLTF(2), :);
            sym       = wlanVHTLTFDemodulate(rec_ltf, cfg);
            chanEst   = wlanVHTLTFChannelEstimate(sym, cfg);
            rec_data  = rx(ind.VHTData(1):ind.VHTData(2), :);
            decodBits = wlanVHTDataRecover(rec_data, chanEst, noiseVarEst, cfg);

            if sum(payl ~= decodBits) == 0
                goodBits = goodBits + length(payl);
            end

        else
            payl      = randi([0,1], cfg.getPSDULength * 8, 1);
            tx        = wlanWaveformGenerator(payl, cfg);
            ind       = wlanFieldIndices(cfg);
            rx        = awgn(tgax(tx), snr_db, 'measured');

            info      = wlanHEOFDMInfo('HE-Data', cfg);
            rxLTF     = rx(ind.HELTF(1):ind.HELTF(2), :);
            demodLTF  = wlanHEDemodulate(rxLTF, 'HE-LTF', cfg);
            chanEst   = wlanHELTFChannelEstimate(demodLTF, cfg);
            rxData    = rx(ind.HEData(1):ind.HEData(2), :);
            demodData = wlanHEDemodulate(rxData, 'HE-Data', cfg);

            chanEstData  = chanEst(info.DataIndices, :, :);
            [eqSym, csi] = wlanHEEqualize( ...
                               demodData(info.DataIndices,:,:), ...
                               chanEstData, noiseVarEst, cfg, 'HE-Data');
            decodBits    = wlanHEDataBitRecover(eqSym, noiseVarEst, csi, cfg);

            if sum(payl ~= decodBits) == 0
                goodBits = goodBits + length(payl);
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