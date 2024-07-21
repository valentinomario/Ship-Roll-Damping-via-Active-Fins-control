function amplitude = findDisturbanceAmplitude(data)
% finds the steady-state disturbance amplitude assuming that
% after half the length of the input paraketer, the transient is
% finished
    amplitude = max(data(floor(length(data)/2):end));
end
