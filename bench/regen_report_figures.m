% regen_report_figures.m - re-export every report figure in MATLAB's light
% theme at print resolution (150 dpi), from saved results where possible.
% In MATLAB:  cd(fullfile(userpath, "fc_test", "bench")); regen_report_figures
set(groot, 'defaultFigureCreateFcn', @(f, ~) lightTheme(f));
set(groot, 'defaultAxesFontSize', 10);
cleanupObj = onCleanup(@() set(groot, 'defaultFigureCreateFcn', ''));
here = fileparts(mfilename('fullpath'));

fprintf('1/4 physics validation figures (V7, V8)\n');
validate_physics();

fprintf('2/4 SITL analysis figure\n');
runDir = fullfile(here, "..", "sitl", "run_20261110_011329");
S = load(fullfile(runDir, "physics_log.mat"));
sitl_analysis(fullfile(runDir, "logs", "00000001.BIN"), S.L);

fprintf('3/4 tune comparison figure\n');
T = load(fullfile(here, "results", "tune_tests.mat"));
compare_tunes(T.res);

fprintf('4/4 benchmark figure\n');
fc_benchmark(FromSaved=true);
fprintf('All report figures regenerated.\n');

function lightTheme(f)
try, theme(f, 'light'); catch, end
f.Color = 'w';
end
