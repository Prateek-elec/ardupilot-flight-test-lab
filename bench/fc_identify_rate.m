function M = fc_identify_rate(w, u, z, dt, seg)
%FC_IDENTIFY_RATE Rate-loop plant from flight data (one axis).
%   M = fc_identify_rate(w, u, z, dt, seg)
%     w   measured body rate (rad/s)          u   rate-controller output (-1..1)
%     z   commanded rate (rad/s)              dt  sample intervals (s)
%     seg flight/segment index per sample (spectra are averaged per segment)
%
% Closed-loop frequency-response identification (indirect method, standard for flight
% test data): with the commanded rate z as the external excitation,
%     P(f) = S_wz(f) / S_uz(f)     (controller output u -> body rate w)
% is unbiased by the feedback. Only frequencies where both z->w and z->u are coherent
% are used. Model fitted to the complex response:
%     P(s) = K e^{-tau s} / ( s (tm s + 1) )   rigid body + first-order motor/ESC lag
% Returns Krad (rad/s^2 per unit), tm, tau, fit (gain fit %), phaseErr, fitVal (mean
% coherence), fband (Hz), nseg, plus the measured response f, P, coh, band.
% (Moved out of fleet_analysis.m unchanged so the tuner app can use it.)
fs = 1 / median(dt); nw = round(4 * fs);
fax = []; Swz = 0; Suz = 0; Szz = 0; Sww = 0; Suu = 0; nseg = 0;
for k = unique(seg)'
    m = seg == k;
    if nnz(m) < 3 * nw, continue; end
    x = detrend(z(m)); y = detrend(w(m)); v = detrend(u(m));
    [a1, fax] = cpsd(x, y, hann(nw), nw / 2, nw, fs);
    a2 = cpsd(x, v, hann(nw), nw / 2, nw, fs);
    Swz = Swz + a1; Suz = Suz + a2;
    Szz = Szz + pwelch(x, hann(nw), nw / 2, nw, fs); Sww = Sww + pwelch(y, hann(nw), nw / 2, nw, fs); Suu = Suu + pwelch(v, hann(nw), nw / 2, nw, fs);
    nseg = nseg + 1;
end
if nseg == 0
    M = struct('f', [], 'P', [], 'coh', [], 'band', [], 'nseg', 0, 'Krad', NaN, 'K', NaN, 'tm', NaN, 'tau', NaN, 'D', 0, ...
        'fit', NaN, 'phaseErr', NaN, 'fitVal', NaN, 'fband', [NaN NaN]);
    return
end
cw = abs(Swz).^2 ./ (Szz .* Sww); cu = abs(Suz).^2 ./ (Szz .* Suu);
P = Swz ./ Suz;
M = struct('f', fax, 'P', P, 'coh', min(cw, cu), 'band', [], 'nseg', nseg);
% K and tm from the gain over 1.5-16 Hz (coherent bins); delay tau from the phase over
% 6-16 Hz only: below ~5 Hz the commanded rate comes from the attitude loop, which reacts
% to the same disturbances, so low-frequency phase is biased.
band = fax >= 1.5 & fax <= 16 & cw >= 0.6 & cu >= 0.6;
if nnz(band) < 6, band = fax >= 1.5 & fax <= 16 & cw >= 0.4 & cu >= 0.4; end
M.band = band;
if nnz(band) < 4
    M.Krad = NaN; M.K = NaN; M.tm = NaN; M.tau = NaN; M.D = 0; M.fit = NaN; M.phaseErr = NaN; M.fitVal = NaN; M.fband = [NaN NaN];
    return
end
fb = fax(band); Pb = P(band); om = 2 * pi * fb;
best = struct('err', inf);
for tm = 0.005:0.001:0.08
    lk = log(abs(Pb)) + log(om) + 0.5 * log(1 + (om * tm).^2);
    K = exp(mean(lk)); err = std(lk);
    if err < best.err, best = struct('err', err, 'K', K, 'tm', tm); end
end
hf = band & fax >= 6;
om6 = 2 * pi * fax(hf);
ph0 = -pi / 2 - atan(om6 * best.tm);
res = angle(exp(1i * (angle(P(hf)) - ph0)));                  % wrapped residual (rad)
tau = max(0, -sum(res .* om6) / sum(om6 .^ 2));
if isempty(om6), tau = 0; end
g = best.K * exp(-1i * om * tau) ./ (1i * om .* (1 + 1i * om * best.tm));
M.Krad = best.K; M.K = rad2deg(best.K); M.tm = best.tm; M.tau = tau; M.D = 0;
M.fit = 100 * (1 - norm(abs(Pb) - abs(g)) / norm(abs(Pb)));        % gain fit
phe = rad2deg(angle(exp(1i * (angle(P(hf)) - angle(best.K * exp(-1i * om6 * tau) ./ (1i * om6 .* (1 + 1i * om6 * best.tm)))))));
M.phaseErr = rms(phe);
M.fitVal = mean(M.coh(band)); M.fband = [min(fb) max(fb)];
end
