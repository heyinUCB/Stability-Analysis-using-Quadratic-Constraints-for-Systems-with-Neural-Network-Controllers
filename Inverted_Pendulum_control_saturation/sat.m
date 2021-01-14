function usat = sat(u, umax, umin)
if u > umax
    usat = umax;
elseif umin <= u && u <= umax
    usat = u;
else
    usat = umin;
end