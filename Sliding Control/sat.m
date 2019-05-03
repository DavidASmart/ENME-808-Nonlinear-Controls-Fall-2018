function [sats] = sat(s, phi)
% saturation function
    if abs(s) < phi
        sats = s/phi;
    else
        sats = sign(s);
    end
end

