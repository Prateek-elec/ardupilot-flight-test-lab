function [ok, note] = fc_check_log(f)
%FC_CHECK_LOG Quick check (seconds) whether an ArduPilot log can be used by the tuner:
%   it must contain RATE messages. The 400 Hz rate and the flight time are checked
%   later, when the tuner reads the full log.
%   [ok, note] = fc_check_log("flight.bin")
try
    lo = ardupilotreader(f);
    av = string(lo.AvailableMessages.MsgName);
    len = seconds(lo.EndTime - lo.StartTime);
catch e
    ok = false; note = "Could not read this file: " + e.message; return
end
if ~any(av == "RATE")
    ok = false; note = "No rate data in this log - it cannot be used. Set LOG_BITMASK = 180223 and fly again.";
elseif ~any(av == "PARM")
    ok = false; note = "No parameters in this log - cannot tell which tune was flying.";
else
    ok = true; note = sprintf("Looks usable (%.0f min log)", len / 60);
end
end
