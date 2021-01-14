function garyfyFigure(fh,opts)
% function garyfyFigure(fh,opts)
%
% Gary-fy figure(s)...
% 
% Inputs:
% -fh: (optional) use the string 'all' or a numerical vector containing figure
% handles. 'all' option Gary-fies all currently open figures. If omitted,
% only the current figure window (gcf) is Gary-fied.
% -opts: (optional) A structure obtained from garyfyFigureOptions()
% containing some of the figure parameters. If omitted, default values in
% the garyfyFigureOptions() is used
%
% AAO 09/14/2011 -Initial coding
if nargin==0
    fh=gcf;
    if isa(fh,'matlab.ui.Figure');
        fh = fh.Number;
    end
    opts=garyfyFigureOptions();
elseif nargin==1
    opts=garyfyFigureOptions();
end

if isstr(fh) && strcmp(fh,'all')
    figHandles = findall(0,'Type','figure');
    for k=1:numel(figHandles)
        garyfyFigure(figHandles(k),opts);
    end
    return;
elseif isnumeric(fh) && isvector(fh) && ~isscalar(fh)
    for k=1:numel(fh)
        garyfyFigure(fh(k),opts);
    end
    return;
elseif isnumeric(fh) && isscalar(fh)
    % just a single call to this function, keep moving forward
else
   error('parameter fh should either be: ''all'' or a numerical vector of figure handles'); 
end    

%% Modify X/Y/ZLabels and figure title for visible axes
h = findall(fh,'Type','axes','Visible','on');
set(h,'FontSize',opts.TickMarkFontSize);
for k=1:numel(h)
    h2=findall(h(k),'Type','Line');
    set(h2,'LineWidth',opts.LineWidth);
    axisFields=get(h(k));
    if isfield(axisFields,'YLabel')
       set(get(h(k),'ZLabel'),'FontSize',opts.AxesLabelFontSize);
       set(get(h(k),'YLabel'),'FontSize',opts.AxesLabelFontSize); 
       set(get(h(k),'XLabel'),'FontSize',opts.AxesLabelFontSize);
       set(get(h(k),'Title'),'FontSize',opts.AxesLabelFontSize);
    end
end    

%% Invisible axes may have visible titles, xlabels etc.
% For instance bode() and bodemag() functions rely on invisible handles to 
% place multiple titles.
h = findall(fh,'Type','axes','Visible','off');
for k=1:numel(h)
    axisFields=get(h(k));
    if isfield(axisFields,'YLabel')
       set(get(h(k),'ZLabel'),'FontSize',opts.AxesLabelFontSize);
       set(get(h(k),'YLabel'),'FontSize',opts.AxesLabelFontSize); 
       set(get(h(k),'XLabel'),'FontSize',opts.AxesLabelFontSize);
       set(get(h(k),'Title'),'FontSize',opts.AxesLabelFontSize);
    end
end    
   
%% Modify the legend
lh = findall(fh,'Type','axes','Tag','legend','Visible','on'); 
set(lh,'Location',opts.LegendLocation); % modify the location of the legend
elh = findall(lh,'Type','hggroup'); % handles to the lines in the legend
tlh = findall(lh,'Type','text'); % handles to the text in the legend
set(tlh,'FontSize',opts.LegendFontSize);
set(cell2mat(get(elh,'Children')),'LineWidth',opts.LineWidth);  % LineWidth for lines in the legend

end