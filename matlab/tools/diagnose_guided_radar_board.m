function report = diagnose_guided_radar_board(takeDir, boardCenterRadar, opts)
%DIAGNOSE_GUIDED_RADAR_BOARD Search raw ADC only inside the known board ROI.
if nargin < 3, opts=struct(); end
o=defaults(opts); matlabDir=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(matlabDir,'algorithm'),'-begin');addpath(fullfile(matlabDir,'algorithm','lib'),'-begin');
loaded=load(fullfile(takeDir,'radar_adc.mat'),'ADCData');adt=loaded.ADCData;per=2048;payloadBytes=numel(adt{1,4});
ids=zeros(size(adt,1),1);for i=1:size(adt,1),v=adt{i,2};if iscell(v),v=v{1};end;ids(i)=double(v(1));end
[sortedIds,sortIndex]=sort(ids);frameIndex=floor(sortedIds/per)+1;[uniqueFrames,~,group]=unique(frameIndex);counts=accumarray(group,1);frames=sort(uniqueFrames(counts==per));
frames=frames(unique(round(linspace(1,numel(frames),min(o.frameCount,numel(frames))))));
expectedRange=norm(boardCenterRadar);expectedAz=atan2d(boardCenterRadar(1),boardCenterRadar(2));expectedEl=atan2d(boardCenterRadar(3),hypot(boardCenterRadar(1),boardCenterRadar(2)));
azGrid=expectedAz+(-o.azimuthHalfWidthDeg:o.angleStepDeg:o.azimuthHalfWidthDeg);
elGrid=expectedEl+(-o.elevationHalfWidthDeg:o.angleStepDeg:o.elevationHalfWidthDeg);
[azimuth,elevation]=meshgrid(azGrid,elGrid);score=zeros(numel(elevation),2*o.rangeBins+1);framePeak=zeros(numel(frames),4);
for index=1:numel(frames)
    raw=assemble(adt,sortedIds,sortIndex,frames(index),per,payloadBytes);
    cube=build_cube(raw);rangeResolution=range_resolution();rangeBin=round((expectedRange+o.expectedRangeBiasM)/rangeResolution)+1;
    bins=max(2,rangeBin-o.rangeBins):min(size(cube,1),rangeBin+o.rangeBins);
    doppler=(65-o.dopplerBins):(65+o.dopplerBins);
    current=zeros(numel(elevation),numel(bins));
    for rangeIndex=1:numel(bins)
        for dopplerIndex=doppler
            vector=squeeze(cube(bins(rangeIndex),dopplerIndex,:));
            current(:,rangeIndex)=current(:,rangeIndex)+guided_spectrum(vector,azimuth(:),elevation(:),(bins(rangeIndex)-1)*rangeResolution);
        end
    end
    score(:,1:numel(bins))=score(:,1:numel(bins))+current;
    [peakPower,linearPeak]=max(current(:));[anglePeak,rangePeak]=ind2sub(size(current),linearPeak);
    framePeak(index,:)=[azimuth(anglePeak) elevation(anglePeak) (bins(rangePeak)-1)*rangeResolution peakPower];
end
score=score/max(numel(frames),1);[peakValue,linearPeak]=max(score(:));[peak,rangePeak]=ind2sub(size(score),linearPeak);medianValue=median(score(:));peakDb=10*log10(peakValue/(medianValue+eps));
peakRange=(bins(rangePeak)-1)*rangeResolution;peakAz=azimuth(peak);peakEl=elevation(peak);point=peakRange*[cosd(peakEl)*sind(peakAz) cosd(peakEl)*cosd(peakAz) sind(peakEl)];
report=struct('takeDir',takeDir,'boardCenterRadar',boardCenterRadar,'expectedRangeM',expectedRange, ...
    'expectedAzimuthDeg',expectedAz,'expectedElevationDeg',expectedEl,'peakAzimuthDeg',peakAz, ...
    'peakElevationDeg',peakEl,'peakRangeM',peakRange,'peakProminenceDb',peakDb,'guidedPointRadar',point, ...
    'errorM',norm(point-boardCenterRadar),'framePeaks',framePeak,'reliable',peakDb>=o.minimumPeakDb);
fprintf('引导板面搜索：期望 r/az/el %.2f/%.2f/%.2f，峰值 %.2f/%.2f/%.2f，突出 %.1f dB，误差 %.0f mm，可靠=%d\n',expectedRange,expectedAz,expectedEl,peakRange,peakAz,peakEl,peakDb,report.errorM*1000,report.reliable);
end

function spectrum=guided_spectrum(vector,azimuth,elevation,range)
rxX=[0 -4 -8 -12 -16 -19 -21 -24];txX=[36 27 18 9 0 -24 -48 -72];txZ=[32 24 16 8 0 0 0 0];
rxCorrection=[0 313 221 21 158 346 225 241];txCorrection=[48 73 193 178 0 212 320 307];
px=zeros(64,1);pz=zeros(64,1);phase=zeros(64,1);for tx=1:8,for rx=1:8,k=(tx-1)*8+rx;px(k)=(txX(tx)+rxX(rx))*1e-3;pz(k)=txZ(tx)*1e-3;phase(k)=txCorrection(tx)+rxCorrection(rx);end,end
calibrated=vector(:).*exp(1i*deg2rad(phase));u=sind(azimuth(:).').*cosd(elevation(:).');w=sind(elevation(:).');projection=px*u+pz*w;quadratic=((px.^2+pz.^2)-projection.^2)/(2*max(range,.3));steering=exp(1i*(-2*pi/(3e8/77e9))*(-projection+quadratic));spectrum=abs(steering'*calibrated).^2;
end
function cube=build_cube(raw)
N=1024;raw=reshape(uint8(raw),2048,2048);adc=double(uint16(raw(1:2:end,:)))+256*double(uint16(raw(2:2:end,:)))-2048;window=single(hamming(N));dopplerWindow=single(hamming(16)).';adc=reshape(adc(:,1025:2048),N,8,16,8);adc=permute(adc,[1 4 2 3]);adc=reshape(adc,N,64,16);rangeFft=fft(single(adc).*reshape(window,N,1,1),N,1);rangeFft=rangeFft(1:N/2,:,:);rd=fft(rangeFft.*reshape(dopplerWindow,1,1,16),128,3);rd=rd(:,:,[65:128 1:64]);cube=permute(rd,[1 3 2]);
end
function value=range_resolution(),value=(3e8/(2*(4e9/60e-6)))*(16.667e6/1024);end
function raw=assemble(adt,ids,index,frame,per,payload),first=find(ids>=(frame-1)*per,1);raw=zeros(payload,per,'uint8');for i=1:per,raw(:,i)=uint8(adt{index(first+i-1),4}(:));end;end
function o=defaults(o),d=struct('frameCount',8,'rangeBins',4,'dopplerBins',1,'expectedRangeBiasM',.14,'azimuthHalfWidthDeg',4,'elevationHalfWidthDeg',4,'angleStepDeg',.25,'minimumPeakDb',3);f=fieldnames(d);for i=1:numel(f),if ~isfield(o,f{i}),o.(f{i})=d.(f{i});end,end;end
