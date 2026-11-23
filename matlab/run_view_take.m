%RUN_VIEW_TAKE Start the multimodal point-cloud player.
%
% Use this script from the MATLAB Editor instead of pressing Run on
% view_take.m itself.  The latter is a function and some MATLAB releases
% have an editor Run regression that exposes matlab.lang.internal.run.FileToRun.
% The player will open the result-selection dialog when no file is supplied.

here = fileparts(mfilename('fullpath'));
addpath(here, '-begin');
view_take();
