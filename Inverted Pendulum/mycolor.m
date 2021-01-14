function rgb = mycolor(name)

if strcmp(name,'maroon')
    rgb = [140 25 25]/256;
end

if strcmp(name,'coolblue')
    rgb = [.4 .4 1];
end

if strcmp(name,'darkgray')
    rgb = [.6 .6 .6];
end

if strcmp(name,'lightgray')
    rgb = [.8 .8 .8];
end

if strcmp(name,'orange')
    rgb = [1 .4 0];
end


end