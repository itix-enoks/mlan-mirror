%% Throughput vs Number of Users: 802.11ac vs 802.11ax
clear; clc; close all;

%% 1. Parameters
chan_coding     = 'LDPC';
delay_profile   = 'Model-A';
tx_rx_distance  = 5;
fixed_snr       = 25;
fixed_mcs       = 4;
fixed_payload   = 1024;
numPackets      = 50;

num_users_list  = [1, 2, 4];

%% 2. Channels
tgax_40 = wlanTGaxChannel(...
    "ChannelBandwidth", 'CBW40', ...
    "DelayProfile", delay_profile, ...
    "TransmitReceiveDistance", tx_rx_distance, ...
    "LargeScaleFadingEffect", 'None');

tgax_80 = wlanTGaxChannel(...
    "ChannelBandwidth", 'CBW80', ...
    "DelayProfile", delay_profile, ...
    "TransmitReceiveDistance", tx_rx_distance, ...
    "LargeScaleFadingEffect", 'None');

noiseVar = 10^(-fixed_snr / 10);

%% 3. Simulation Sweep
tp_ac = zeros(1, length(num_users_list));
tp_ax = zeros(1, length(num_users_list));

for i = 1:length(num_users_list)
    n = num_users_list(i);
    fprintf('Simulating %d users...\n', n);

    if n <= 2
        cbw  = 'CBW40';
        tgax = tgax_40;
    else
        cbw  = 'CBW80';
        tgax = tgax_80;
    end

    % Reset channel at the start of each user-count experiment
    reset(tgax);

    %% --- 802.11ac (VHT): round-robin time slicing ---
    cfg_ac = wlanVHTConfig(...
        "ChannelBandwidth", cbw, ...
        "MCS", fixed_mcs, ...
        "ChannelCoding", chan_coding, ...
        "APEPLength", fixed_payload);

    fs_ac = wlanSampleRate(cfg_ac);
    total_bits_ac = 0;
    total_time_ac = 0;

    for pkt = 1:numPackets
        payload = randi([0 1], cfg_ac.PSDULength * 8, 1);
        tx = wlanWaveformGenerator(payload, cfg_ac);
        pkt_duration = size(tx, 1) / fs_ac;

        reset(tgax);
        faded  = tgax(tx);
        rx     = awgn(faded, fixed_snr, 'measured');

        ind     = wlanFieldIndices(cfg_ac);
        ltf     = rx(ind.VHTLTF(1):ind.VHTLTF(2), :);
        sym     = wlanVHTLTFDemodulate(ltf, cfg_ac);
        chanEst = wlanVHTLTFChannelEstimate(sym, cfg_ac);
        rxData  = rx(ind.VHTData(1):ind.VHTData(2), :);
        rxBits  = wlanVHTDataRecover(rxData, chanEst, noiseVar, cfg_ac);

        if ~any(payload ~= rxBits)
            total_bits_ac = total_bits_ac + cfg_ac.PSDULength * 8;
        end
        % Round-robin: full round time = n slots
        total_time_ac = total_time_ac + n * pkt_duration;
    end

    tp_ac(i) = total_bits_ac / total_time_ac;

    %% --- 802.11ax (HE): all users simultaneously via OFDMA ---
    if n == 1
        cfg_ax = wlanHESUConfig(...
            "ChannelBandwidth", cbw, ...
            "MCS", fixed_mcs, ...
            "ChannelCoding", chan_coding, ...
            "APEPLength", fixed_payload);

        fs_ax = wlanSampleRate(cfg_ax);
        total_bits_ax = 0;
        total_time_ax = 0;

        for pkt = 1:numPackets
            payload = randi([0 1], cfg_ax.getPSDULength * 8, 1);
            tx = wlanWaveformGenerator(payload, cfg_ax);
            pkt_duration  = size(tx, 1) / fs_ax;
            total_time_ax = total_time_ax + pkt_duration;

            reset(tgax);
            faded = tgax(tx);
            rx    = awgn(faded, fixed_snr, 'measured');

            ind  = wlanFieldIndices(cfg_ax);
            info = wlanHEOFDMInfo('HE-Data', cfg_ax);

            rxLTF    = rx(ind.HELTF(1):ind.HELTF(2), :);
            demodLTF = wlanHEDemodulate(rxLTF, 'HE-LTF', cfg_ax);
            chanEst  = wlanHELTFChannelEstimate(demodLTF, cfg_ax);
            rxData   = rx(ind.HEData(1):ind.HEData(2), :);
            demodData = wlanHEDemodulate(rxData, 'HE-Data', cfg_ax);

            chanEstData  = chanEst(info.DataIndices, :, :);
            [eqSym, csi] = wlanHEEqualize(demodData(info.DataIndices, :, :), chanEstData, noiseVar, cfg_ax, 'HE-Data');
            rxBits       = wlanHEDataBitRecover(eqSym, noiseVar, csi, cfg_ax);

            if ~any(payload ~= rxBits)
                total_bits_ax = total_bits_ax + cfg_ax.getPSDULength * 8;
            end
        end

    else
        if n == 2
            alloc = [192 192];
        else
            alloc = [192 192 192 192];
        end

        cfg_ax = wlanHEMUConfig(alloc);
        for u = 1:n
            cfg_ax.User{u}.MCS           = fixed_mcs;
            cfg_ax.User{u}.ChannelCoding = chan_coding;
            cfg_ax.User{u}.APEPLength    = fixed_payload;
        end

        fs_ax = wlanSampleRate(cfg_ax);
        total_bits_ax = 0;
        total_time_ax = 0;

        for pkt = 1:numPackets
            payloads = cell(1, n);
            for u = 1:n
                payloads{u} = int8(randi([0 1], cfg_ax.User{u}.APEPLength * 8, 1));
            end

            tx = wlanWaveformGenerator(payloads, cfg_ax);
            pkt_duration  = size(tx, 1) / fs_ax;
            total_time_ax = total_time_ax + pkt_duration;

            reset(tgax);
            faded = tgax(tx);
            rx    = awgn(faded, fixed_snr, 'measured');
            ind   = wlanFieldIndices(cfg_ax);

            for ru = 1:numel(cfg_ax.RU)
                userIdx = cfg_ax.RU{ru}.UserNumbers;
                info    = wlanHEOFDMInfo('HE-Data', cfg_ax, ru);

                rxLTF    = rx(ind.HELTF(1):ind.HELTF(2), :);
                demodLTF = wlanHEDemodulate(rxLTF, 'HE-LTF', cfg_ax, ru);
                chanEst  = wlanHELTFChannelEstimate(demodLTF, cfg_ax, ru);

                rxData    = rx(ind.HEData(1):ind.HEData(2), :);
                demodData = wlanHEDemodulate(rxData, 'HE-Data', cfg_ax, ru);

                chanEstData  = chanEst(info.DataIndices, :, :);
                [eqSym, csi] = wlanHEEqualize(demodData(info.DataIndices, :, :), chanEstData, noiseVar, cfg_ax, 'HE-Data', ru);
                rxBits       = wlanHEDataBitRecover(eqSym, noiseVar, csi, cfg_ax, userIdx);

                if isequal(payloads{userIdx}, rxBits)
                    total_bits_ax = total_bits_ax + cfg_ax.User{userIdx}.APEPLength * 8;
                end
            end
        end
    end

    tp_ax(i) = total_bits_ax / total_time_ax;
end

%% 4. Plot
figure;
plot(num_users_list, tp_ac/1e6, 'r-o', 'LineWidth', 2, 'MarkerSize', 8); hold on;
plot(num_users_list, tp_ax/1e6, 'b-s', 'LineWidth', 2, 'MarkerSize', 8);
grid on;
xlabel('Number of Concurrent Users');
ylabel('Aggregate Throughput (Mbps)');
title('Throughput vs Users: 802.11ac vs 802.11ax');
legend('802.11ac (Round-Robin TDMA)', '802.11ax (OFDMA MU)', 'Location', 'best');
xticks(num_users_list);