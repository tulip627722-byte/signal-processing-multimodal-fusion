function path = dump_orbbec_factory_calibration(path)
%DUMP_ORBBEC_FACTORY_CALIBRATION Export RGB/depth intrinsics and extrinsics.
here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(path)
    path = fullfile(here, '..', 'capture', 'app', 'orbbec_factory_calibration.json');
end
path = char(path);
executable = fullfile(here, '..', 'capture', 'app', 'bin', 'Acquire.exe');
if ~isfile(executable), error('找不到采集程序：%s', executable); end
command = sprintf('"%s" --dump-factory-calibration --out "%s"', executable, path);
[status, output] = system(command);
if status ~= 0 || ~isfile(path)
    error('Orbbec 工厂标定导出失败：%s', strtrim(output));
end
load_orbbec_factory_calibration(path);
fprintf('Orbbec 工厂标定已保存并通过校验：%s\n', path);
end
