function F = lab_campaign_data()
%LAB_CAMPAIGN_DATA  Load fleet_results/fleet.mat, the identified data of the original test campaign.
%
%   That file comes from the author's own flight logs, which are not part of the repository,
%   so the scripts that replay the campaign (lqr_pid_tuner with no data, pid_design_space,
%   fleet_margins, fleet_figures) explain what to do instead of failing on a missing file.

f = fullfile(fileparts(fileparts(mfilename('fullpath'))), "fleet_results", "fleet.mat");
if ~isfile(f)
    error('lab:noCampaignData', ['%s not found.\n' ...
        'This script replays the original test campaign, whose flight logs are not included.\n' ...
        'To tune YOUR aircraft, run  fc_tune_from_log("your_flight.bin")  or open  fc_tuner_app :\n' ...
        'they identify your airframe from the log and run the same LQR design on it.\n' ...
        'To rebuild fleet.mat from your own 400 Hz logs, list them in bench/fleet_analysis.m and run it.'], f);
end
S = load(f, 'F');
F = S.F;
end
