
%{
Geometry.BSPos = [ 0,0,25 ];
N_ant = input('inserire il numero di antenne lungo x e y: ');                                   %number of antennas
N_tx= input('inserire il numero di tx: ');
N_int=input('inserire il numero di interf: ');
N_symbols = input('inserire il numero di simboli inviati: ');                             %number of OFDM symbols
M=input('inserire modulation order: ');
Pars.SNR_dB = input('inserisci il valore snr iniziale: ');
%dichiarazione variabili
Geometry.VPosStart=zeros(N_tx,3);
Geometry.VPosEnd=zeros(N_tx,3);
Geometry.T=zeros(N_tx);
Geometry.DistVStart=zeros(N_tx);
Geometry.IPos=zeros(N_int,3);
 
for i=1:N_tx
 
    fprintf('inserisci la coordinata iniziale x del veicolo %d', i);
    Geometry.VPosStart(i,1)=input(' ');
 
    fprintf('inserisci la coordinata iniziale y del veicolo %d', i);
    Geometry.VPosStart(i,2)=input(' ');
 
    fprintf('inserisci la coordinata iniziale z del veicolo %d', i);
    Geometry.VPosStart(i,3)=input(' ');
 
    fprintf('inserisci la coordinata finale x del veicolo %d', i);
    Geometry.VPosEnd(i,1)=input(' ');
 
    fprintf('inserisci la coordinata finale y del veicolo %d', i);
    Geometry.VPosEnd(i,2)=input(' ');
 
    fprintf('inserisci la coordinata finale z del veicolo %d', i);
    Geometry.VPosEnd(i,3)=input(' ');
 
    Geometry.T(i) = sqrt(sum((Geometry.VPosEnd(i,1:2)-Geometry.VPosStart(i,1:2)).^2)); %Distance travelled by Vehicle i
    Geometry.DistVStart(i) = sqrt(sum((Geometry.VPosStart(i,1:2)-Geometry.BSPos(1,1:2)).^2))  %Distance between Vehicle 1 and BS
end
 
for i=1:N_int
    fprintf('inserisci la coordinata x del segnale interferente %d', i);
    Geometry.IPos(i,1)=input(' ');
 
    fprintf('inserisci la coordinata y del segnale interferente %d', i);
    Geometry.IPos(i,2)=input(' ');
 
    fprintf('inserisci la coordinata z del segnale interferente %d', i);
    Geometry.IPos(i,3)=input(' ');
end
%}

N_ant=8;
N_tx=2;
N_int=2;
N_symbols=100;
M=4;
Pars.SNR_dB = 20;
study_channel = 1;                            %channel we want to study
study_antenna = 1;                            
study_Track = 1;                              


%% GEOMETRY
%memory allocation
Geometry.BSPos = [ 0,0,25 ];                 %base station position
Geometry.VPosStart=zeros(N_tx,3);            %starting position of the transmitters
Geometry.VPosEnd=zeros(N_tx,3);              %end position of the transmitters
Geometry.T=zeros(N_tx);                      %traveled distance for trasmitters
Geometry.DistVStart=zeros(N_tx);             %initial distance
Geometry.IPos=zeros(N_int,3);                %position of interferences, only one since they are static
 
Geometry.VPosStart(1,1)=70;
Geometry.VPosStart(1,2)=-100;
Geometry.VPosStart(1,3)=1.5;
Geometry.VPosEnd(1,1)=70;
Geometry.VPosEnd(1,2)=100;
Geometry.VPosEnd(1,3)=1.5;
 
Geometry.VPosStart(2,1)=200;
Geometry.VPosStart(2,2)=-50;
Geometry.VPosStart(2,3)=1.5;
Geometry.VPosEnd(2,1)=10;
Geometry.VPosEnd(2,2)=-50;
Geometry.VPosEnd(2,3)=1.5;
 
Geometry.IPos(1,1)=10;
Geometry.IPos(1,2)=-210;
Geometry.IPos(1,3)=1.5;

Geometry.IPos(2,1)=-150;
Geometry.IPos(2,2)=100;
Geometry.IPos(2,3)=1.5;
 
for i=1:N_tx
    Geometry.T(i) = sqrt(sum((Geometry.VPosEnd(i,1:2)-Geometry.VPosStart(i,1:2)).^2));           %Distance travelled by Vehicle i
    Geometry.DistVStart(i) = sqrt(sum((Geometry.VPosStart(i,1:2)-Geometry.BSPos(1,1:2)).^2))     %Distance between Vehicle 1 and Base station
end

N_disp_tot=N_tx+N_int;                            % total devices number
N_subcarriers = 3+2*(N_disp_tot);                 % number of OFDM subcarriers
N_eff_subcarriers = N_subcarriers-2-1;            %a pilot frequency each, plus 2 guardbands it's the number of effectively used subcarriers

pilot=zeros(1, N_disp_tot);
for i=1:N_disp_tot
    pilot(1,i)=1+2*i;
end
 
Pars.fc = 2.6e9;                                %Carrier frequency
Pars.c = physconst('LightSpeed');               %C constant
Pars.lambda = Pars.c/Pars.fc;                   %wavelenght
 
%Defining a NxN antenna array
Geometry.BSarray = phased.URA('Size', [ N_ant N_ant ], 'ElementSpacing', ...
    [ Pars.lambda/2 Pars.lambda/2 ], 'ArrayNormal', 'z');
Geometry.BSAntennaPos = getElementPosition(Geometry.BSarray);       
 
 
 
%% QUADRIGA
%launch quadriga
l = qd_layout;
l.set_scenario('QuaDRiGa_UD2D_LOS');     %UMa menans urban macro area
 
l.simpar.center_frequency = Pars.fc;
txArr = qd_arrayant('omni');
rxArr = qd_arrayant('omni');
rxArr.no_elements = N_ant * N_ant;
rxArr.element_position = Geometry.BSAntennaPos;
 

l.tx_array = txArr;
l.rx_array = rxArr;
l.no_rx = 1; %1 Base Station
l.no_tx = N_disp_tot; 

for i=1:N_tx
    angle=atan((Geometry.VPosEnd(i,2)-Geometry.VPosStart(i,2))/(Geometry.VPosEnd(i,1)-Geometry.VPosStart(i,1)));        %calculate the angle between transmitter and x-positive depending on movement direction
    if Geometry.VPosEnd(i,1)<Geometry.VPosStart(i,1)                                                                    %if we move in the other direction we add pi
        angle=angle+pi;
    end
    angle;
    tx_track(i) = qd_track('linear', Geometry.T(i),angle);                                                              
    tx_track(i).name = ['trackV', num2str(i)];
end
 
for i=1:N_tx
    tx_track(i).initial_position = Geometry.VPosStart(i,:)';
end
 
l.tx_position = [ Geometry.VPosStart', Geometry.IPos'];                                                                 %now we use l. to access because it's in the quadriga structure
for i=1:N_tx
    tx_track(i).movement_profile=[0,10;0,100];
    tx_track(i).interpolate_positions(0.1);                                                                              %one position every 10 meters so we dont take too many points for tx_track
    l.tx_track(1,i)=copy(tx_track(i));
end         
l.rx_position = Geometry.BSPos';            

 %l.visualize();        
 %view(45, 45);        
 
l.set_pairing; 
chan = l.get_channels; 
 
%% OFDM Waveform

    % input bit source:
in = randi([0 1], N_eff_subcarriers*N_symbols*log2(M), N_disp_tot);                %matrice di 4 righe e sulle righe stanno i bit, generati randomicamente, il numero di simboli è 1100 (11 subcarrier*100simboli)
clear ofdmMod
clear waveform
for i=1:N_disp_tot
    % OFDM configuration:
    ofdmMod{i} = comm.OFDMModulator('FFTLength', N_subcarriers, ...                                                     %this part was generated using the waveform tool
        'NumGuardBandCarriers', [1;1], ...
        'InsertDCNull', false, ...
        'CyclicPrefixLength', [0], ...
        'Windowing', false, ...
        'NumSymbols', N_symbols, ...
        'NumTransmitAntennas', 1, ... 
        'PilotInputPort', true, ...
        'PilotCarrierIndices', pilot(i));


    

    dataInput = qammod(in(:,i), M, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
    ofdmInfo = info(ofdmMod{i});
    ofdmSize = ofdmInfo.DataInputSize;
    dataInput = reshape(dataInput, ofdmSize);

    % waveform generation:
    pilotInput = ones(1,N_symbols,1);
    waveform(:,i) = ofdmMod{i}(dataInput, pilotInput);
    
    ofdmDemod{i}=comm.OFDMDemodulator(ofdmMod{i});
end 


Fs = 1.1e+07; 								 % sample rate of waveform
 
waveform=waveform.';


%{
%% Visualize OFDM Waveform
% Spectrum Analyzer
spectrum = dsp.SpectrumAnalyzer('SampleRate', Fs);
spectrum(waveform.');
release(spectrum);
 
% OFDM Subcarrier Mapping
showResourceMapping(ofdmMod); 
%}
 

%% OUTPUT
 
TS = 1e-3;                                  %sampling time
WFlength = size(waveform);
TsVect = 0:TS:TS*(WFlength(2)-1);
     
   for chan_number = 1:N_tx                  %for cycle for the transmitters
        waveform1 = waveform(chan_number,:);
        chTaps = size(chan(chan_number).delay);
       cha_length = length(chan(chan_number).delay);                                         % represent the convolution of the input signal with the channel response. At the end
                                                                                             %of the process, we get a new 4-d matrix called ChanOutput. Here we find the 4
                                                                                             %channels as first dimension, the 20 track points as second dimension, the 64
                                                                                             %antennas as third dimension and 1100 symbols transmitted.
       track_vec = 1:size(chan(chan_number).delay,4);
      for track = 1:1:(length(track_vec)-1)        %for each point of the track
          chOut = zeros(chTaps(1), WFlength(2));   
          for antenna = 1:1:chTaps(1)              %for each antenna
              for path = 1:1:chTaps(3)       
                  inX = TsVect-chan(chan_number).delay(antenna,1,path,track_vec(track));  
                  inY = interp1(TsVect,waveform1,inX,'pchip');
                  chOut(antenna,:) = inY*chan(chan_number).coeff(antenna,1,path,track_vec(track)) ...
                      +chOut(antenna,:); 
              end 
          end
          ChanOutput(chan_number, track, :, :) = chOut;
          Real_DOA(chan_number, track, :) = functionDOA(Geometry.BSPos, chan(chan_number).tx_position(:, track_vec(track)));
 
      end
   end
 
 
  for chan_number = (N_tx+1):N_disp_tot     %for cycle for the fixed interferences
 
    waveform1 = waveform(chan_number,:);
    chTaps = size(chan(chan_number).delay);       
       chOut = zeros(chTaps(1), WFlength(2));
       for antenna = 1:1:chTaps(1)       
          for path = 1:1:chTaps(3) 
              inX = TsVect-chan(chan_number).delay(antenna,1,path,1);
              inY = interp1(TsVect,waveform1,inX,'pchip');
              chOut(antenna,:) = inY*chan(chan_number).coeff(antenna,1,path,1) ...
                  +chOut(antenna,:); 
          end    
       end
       Real_DOA(chan_number, 1, :) = functionDOA(Geometry.BSPos, chan(chan_number).tx_position(:, 1));
 
       for track = 1:(length(track_vec)-1)
           ChanOutput(chan_number, track, :, :) = chOut;
 
       end   
   end
    lgth = length(in(:,study_channel));
 %}
%study_channel=2;
%% CONVENTIONAL BEAMFORMING
 
close all

ChannelOutput_sum = sum(ChanOutput,[1]); %summing the waveforms
ChannelOutput_reshape = reshape(ChannelOutput_sum(1,study_Track,:,:), [rxArr.no_elements, WFlength(2)]);
ChannelOutput_awgn = zeros(rxArr.no_elements, WFlength(2)); 
ChannelOutput_awgn = awgn(ChannelOutput_reshape,Pars.SNR_dB,'measured');
ChannelOutput_total = transpose(ChannelOutput_awgn);


% the estimator returns 180+true_angle because is defined with respect to the BS considering the negative  direction instead of the positive direction

%DOAS estimation with MUSIC
estimator = phased.MUSICEstimator2D('SensorArray',Geometry.BSarray,...
    'OperatingFrequency',Pars.fc,...
    'ForwardBackwardAveraging',true,...
    'NumSignalsSource','Property',...
    'DOAOutputPort',true,...     
    'NumSignals',l.no_tx,...      
    'AzimuthScanAngles',-180:0.5:180,...     
    'ElevationScanAngles',0:0.5:90);
[a,doas_est] = estimator(ChannelOutput_total);

figure(1)
plotSpectrum(estimator);


angle=Real_DOA(study_channel,study_Track,:)
angle=reshape(angle, [2,1]);
angle(2,1)=angle(2,1);
angle1=zeros(2,N_disp_tot);

for it=1:N_disp_tot
   angle1(1,it)= Real_DOA(it,study_Track,1);
   angle1(2,it)= Real_DOA(it,study_Track,2);
end
%Conventional beamforming
    beamformer = phased.PhaseShiftBeamformer('SensorArray',Geometry.BSarray,...
        'OperatingFrequency', Pars.fc,...
        'Direction', angle,...       
        'WeightsOutputPort', true);
    [arrOut,w_bf] = beamformer(ChannelOutput_total); 
     
    %Scattergraphs wo BF
    output_without = ofdmDemod{study_channel}(ChannelOutput_total(:,study_antenna)); 
    x_wo = real(output_without);
    x_wo = reshape(output_without,[N_eff_subcarriers*N_symbols,1]);
    y_wo = imag(output_without);
    y_wo = reshape(y_wo, [N_eff_subcarriers*N_symbols,1]);      
 
    %Scattergraph with c BF
    output_CB = ofdmDemod{study_channel}(arrOut(:,study_antenna));
    x_CB = real(output_CB);
    x_CB = reshape(x_CB, [N_eff_subcarriers*N_symbols,1]);
    y_CB = imag(output_CB);
    y_CB = reshape(y_CB, [N_eff_subcarriers*N_symbols,1]);
 
    %Pattern position
     figure(2)
     pattern(Geometry.BSarray, Pars.fc, [-180:180], 0, ...
     'CoordinateSystem', 'rectangular', ...
     'Type', 'powerdb', 'PropagationSpeed', physconst('LightSpeed'),...
     'Weights', w_bf);
     
 
 
 
    %%  COMPUTATION OF RX
 
    %Signals power
    Power_signal = (rms(waveform(study_channel,:)).^2);
 
    %Noise power 
    Pars.SNR_dec = 10^(Pars.SNR_dB/10);     %Linear SNR
    Power_noise = Power_signal/Pars.SNR_dec;       %Sigma squared
 
    %Steering matrix for a rectangular array
    steervec = phased.SteeringVector('SensorArray',Geometry.BSarray, ...
        'PropagationSpeed',physconst('LightSpeed'));
    S = steervec(Pars.fc,angle1);
    
    %Sources autocorrelation of the inputs 
    U = Power_signal * eye(l.no_tx); %Power sig * Identity matrix of order l.no_tx
 
    %Autocorrelation of the inputs
    Rx = S*U*S';        
    Rx_noise = Rx + Power_noise * eye(rxArr.no_elements);
 
 %}
   %% LMS
    %Output signal 
    y = ChanOutput(study_channel,study_Track,:,:); 
    y=reshape(y,[antenna,WFlength(2)]);
    d = awgn(y,Pars.SNR_dB,'measured');
    
     %Desired signal
    x = waveform(study_channel,:);      
    x = reshape(x,[1,WFlength(2)]);
   
    
    h_tilde = zeros(rxArr.no_elements,1);
    y_tilde=zeros(antenna,1);
    err=zeros(antenna,1);
    mu = 1/trace(Rx_noise); %the first step we want (usually  it's the best choice for LMS)
    order_filter=10;        %sequence of bits that we take from the input to estimate the antenna coefficients 
    output_LMS=zeros(5,20);
    
    %}
    %LMS implementation where we make better estimations by adding to the
    % previously calculated the product of mu times the conjugate of the
    % error
   
         for n = 1:order_filter
             y_tilde=conj(h_tilde)'*x(1,n); %product of h Hermitian by x(n)
             err(:,1) = d(:,n)-y_tilde';  %error estimation
             h_tilde = h_tilde + mu * conj(err)*x(n);
         end
 
    figure(2)
     pattern(Geometry.BSarray, Pars.fc, [-180:180], 0, ...
     'CoordinateSystem', 'rectangular', ...
     'Type', 'powerdb', 'PropagationSpeed', physconst('LightSpeed'),...
     'Weights', h_tilde);
    %Scattergraph with LMS BEAFORMING
    
    output_LMS = conj(h_tilde)'*d;
    output_LMS = ofdmDemod{study_channel}(output_LMS.');
    x_LMS = real(output_LMS);
    x_LMS = reshape(x_LMS, [N_eff_subcarriers*N_symbols,1]);
    y_LMS = imag(output_LMS);
    y_LMS = reshape(y_LMS, [N_eff_subcarriers*N_symbols,1]);
    
    %% SCATTERGRAPHS
 
     figure(5)
     hold on
     subplot(1,3,1)
     scatter(x_wo,y_wo); title('Constellation without beamforming');
     subplot(1,3,2)
     scatter(x_CB,y_CB); title('Constellation with conventional beamforming - CB - ');
     subplot(1,3,3)
     scatter(x_LMS,y_LMS); title('Constellation with LMS beamforming');
  
 
 
   %% BER
   %BER Without beamforming 
    out_without = qamdemod(output_without,M,'gray', 'OutputType', 'bit', 'UnitAveragePower', true); %18,140
    out_without = reshape(out_without,[lgth,1]);
    [errors_WO,BER_WO] = biterr(in(:,study_channel), out_without);
 
    %With conventional beamforming using ofdmdemod and qamdemod
    BER_CB=1;
    it=0;
    while (BER_CB>=0.05)
    out_CB_def = qamdemod(output_CB,M,'gray', 'OutputType', 'bit', 'UnitAveragePower', true);
    output_CB=output_CB*exp(-1i*pi/2*it);    
    out_CB_def = reshape(out_CB_def,[lgth,1]);
    [errors_CB,BER_CB] = biterr(in(:,study_channel), out_CB_def);
    it=it+1;
        if it>=5
            break;
        end
    end
    BER_CB
 
    %With LMS 
    out_LMS = qamdemod(output_LMS,M,'gray', 'OutputType', 'bit', 'UnitAveragePower', true);
    out_LMS = reshape(out_LMS,[lgth,1]);
    [errors_LMS,BER_LMS] = biterr(in(:,study_channel), out_LMS);
 
    BER_LMS

%% FUNCTION DOA
function [DOA] = functionDOA(rx,tx)

tx = tx';
dist = sqrt(sum((tx(1,1:2)-rx(1,1:2)).^2));
AOA = atan((rx(1,2) - tx(1,2))/(rx(1,1) - tx(1,1)))*180/pi; 
EOA = 90 - atan2(dist, rx(1,3) - tx(1,3))*180/pi;
DOA = [AOA EOA];
 
end
 