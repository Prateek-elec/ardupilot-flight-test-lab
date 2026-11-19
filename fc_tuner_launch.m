function fc_tuner_launch()
%FC_TUNER_LAUNCH Entry point used by "FC Tuner.exe": opens the tuner window and quits
%MATLAB when the window is closed. (In a normal MATLAB session use fc_tuner_app instead.)
try
    app = fc_tuner_app();
    waitfor(app.Figure);
catch e
    f = uifigure('Name', 'FC PID Tuner', 'Position', [300 300 520 160]);
    uialert(f, e.message, 'The tuner could not start', 'CloseFcn', @(~, ~) delete(f));
    waitfor(f);
end
exit;
end
