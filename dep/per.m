%% Initial parameters: These are the ranges / values we can consider
cbw = {'CBW20', 'CBW40', 'CBW80', 'CBW160'};    % Channel Bandwidth
num_usr = 1:4;                                  % num users
mcs_ac = 0:9;                                   % MCS ac
mcs_ax = 0:11;                                  % MCS ax

chan_cod = {'BCC', 'LDPC'};                  % Channel coding
guard_i = {'short', 'long'};                    % Guard interval ac
guard_i_ax = {3.2, 1.6, 0.8};                   % Guard interval ax
payload_len = {1024, 200};                      % Payload len: video / audio

snr_ac = [3, 4, 5, 7, 9, 12, 16, 20, 25, 30];            % SNR ac
snr_ax = [3, 4, 5, 7, 9, 12, 16, 20, 25, 30];            % SNR ax

% transmission
del_prof = {'Model-A', 'Model-B', 'Model-C' ,'Model-D' 'Model-E' 'Model-F'};
tx_rx_dist = {1, 3, 5, 7, 10, 15};              % in meters
trans_dir = {'Downlink', 'Uplink'};
lscale_fading = {'None', 'Pathloss', 'Shadowing', 'Pathloss and shadowing'};


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Fill in these parameters to use different settings
chan_bandwidth = cbw{2};
chan_coding = chan_cod{2};
delay_profile = del_prof{3};
tx_rx_distance = tx_rx_dist{3};
transmission_dir = trans_dir{1};
large_scale_fading_effects = lscale_fading{3};

num_user_context = 5;
num_user_ac = num_usr(1);
user_to_analyze = 2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% EXPERIMENT RUN
% Create TGax channel
tgax = genTGax(chan_bandwidth, delay_profile, tx_rx_distance, transmission_dir, large_scale_fading_effects);
% Create user Configs
[user_ac, user_ax] = genUserCfg(num_user_context, chan_bandwidth, num_user_ac, mcs_ac, mcs_ax, chan_coding, guard_i, guard_i_ax, payload_len);

% Calc PER
[PER_AC, PER_AX] = calcPerForSnrVals(user_ac{user_to_analyze}, user_ax{user_to_analyze}, snr_ac, snr_ax, tgax);

%% Figure
PER_AC(PER_AC == 0) = 1e-6;
PER_AX(PER_AX == 0) = 1e-6;

figure;
semilogy(snr_ac, PER_AC, 'r-o', 'LineWidth', 2); hold on;
semilogy(snr_ax, PER_AX, 'b-s', 'LineWidth', 2);
grid on;
xlabel('SNR (dB)');
ylabel('Packet Error Rate (PER)');
legend('802.11ac', '802.11ax');
title('WiFi Performance Comparison');




function [users_ac, users_ax] = genUserCfg(num_cfg, chan_bw, num_usr_ac, mcs_ac, mcs_ax, chan_coding, guard_i, guard_i_ax, payload_len)
    user_ac = cell(1, num_cfg);
    user_ax = cell(1, num_cfg);

    for i = 1:num_cfg
        % ac
        user_ac{i} = wlanVHTConfig("ChannelBandwidth", chan_bw, ...
            "NumUsers", num_usr_ac, ...
            "MCS", mcs_ac(i), ...
            "ChannelCoding", chan_coding, ...
            "GuardInterval", guard_i{ 2 - mod(i, 2) }, ...
            "APEPLength", payload_len{ 2 - mod(i, 2) });
    
        % ax
        guard_i_ax_ind = 1;
        user_ax{i} = wlanHESUConfig("ChannelBandwidth", chan_bw, ...
            "MCS", mcs_ax(i), ...
            "ChannelCoding", chan_coding, ...
            "GuardInterval", guard_i_ax{guard_i_ax_ind}, ...
            "APEPLength", payload_len{ 2 - mod(i, 2) });
        
        gi_val = guard_i_ax{ guard_i_ax_ind };
        if gi_val == 3.2 || gi_val == 1.6 || gi_val == 0.8
            user_ax{i}.HELTFType = gi_val / 0.8;
        end
    end

    users_ac = user_ac;
    users_ax = user_ax;
end

% Fading channel
function tgax = genTGax(cbw, delp, txrxdist, trans_dir, ls_fad_eff)
    tgax = wlanTGaxChannel("ChannelBandwidth", cbw, ...
        "ChannelFiltering", 1, ...
        "DelayProfile", delp, ...
        "TransmitReceiveDistance", txrxdist, ...
        "TransmissionDirection", trans_dir, ...
        "LargeScaleFadingEffect", ls_fad_eff);
end

% Calculate PER over various samples for different values of SNR
function [per_arr_ac, per_arr_ax] = calcPerForSnrVals(user_ac, user_ax, snr_ac, snr_ax, tgax)

    numPackets = 50;
    per_ac = zeros(size(snr_ac));
    per_ax = zeros(size(snr_ax));
    
    for step = 1:length(snr_ac)
        snrCurrent = snr_ac(step);
        noiseVarEst = linearSNR(snrCurrent);
    
        packetErrors_ac = 0;
        packetErrors_ax = 0;
    
        for packet = 1:numPackets
    
            % Create payload
            payl_ac = generatePayloadAc(user_ac);
            payl_ax = generatePayloadAx(user_ax);
    
            %% Transmission, fading and reception for ac
            TxWaveForm = wlanWaveformGenerator(payl_ac, user_ac);
            fadedSignal = tgax(TxWaveForm);
            rx = awgn(fadedSignal, snrCurrent, 'measured');
            
            ind = wlanFieldIndices(user_ac);
            rec_data = rx(ind.VHTLTF(1):ind.VHTLTF(2), :);
            sym = wlanVHTLTFDemodulate(rec_data, user_ac);
            chanEst = wlanVHTLTFChannelEstimate(sym, user_ac);
            rec_data  = rx(ind.VHTData(1):ind.VHTData(2), :);
            decodBits = wlanVHTDataRecover(rec_data, chanEst, noiseVarEst, user_ac);
            
            packetErrors_ac = packetErrors_ac + any(payl_ac ~= decodBits);
    
            %% Transmission, fading and reception for ax
            TxWaveForm = wlanWaveformGenerator(payl_ax, user_ax);            
            fadedSignal = tgax(TxWaveForm);            
            rx = awgn(fadedSignal, snrCurrent, 'measured');
            
            ind = wlanFieldIndices(user_ax);
            info = wlanHEOFDMInfo('HE-Data', user_ax);

            rxLTF = rx(ind.HELTF(1):ind.HELTF(2), :);
            demodLTF = wlanHEDemodulate(rxLTF, 'HE-LTF', user_ax);
            chanEst = wlanHELTFChannelEstimate(demodLTF, user_ax);

            rxData = rx(ind.HEData(1):ind.HEData(2), :);
            demodData = wlanHEDemodulate(rxData, 'HE-Data', user_ax);
            
            chanEstData = chanEst(info.DataIndices, :, :);
            [eqSym, csi] = wlanHEEqualize(demodData(info.DataIndices, :, :), chanEstData, noiseVarEst, user_ax, 'HE-Data');
            decodBits = wlanHEDataBitRecover(eqSym, noiseVarEst, csi, user_ax);

            packetErrors_ax = packetErrors_ax + any(payl_ax ~= decodBits);
        end

        per_ac(step) = packetErrors_ac / numPackets;
        per_ax(step) = packetErrors_ax / numPackets;
    end

    per_arr_ac = per_ac;
    per_arr_ax = per_ax;
end

function payl_data = generatePayloadAc(config)
    payl_bits = config.PSDULength * 8;
    payl_data = randi([0,1], payl_bits, 1);
end

function payl_data = generatePayloadAx(config)
    payl_bits = config.getPSDULength * 8;
    payl_data = randi([0,1], payl_bits, 1);
end

function snr = linearSNR(snr)
    snr = 10 ^ (-snr / 10);
end