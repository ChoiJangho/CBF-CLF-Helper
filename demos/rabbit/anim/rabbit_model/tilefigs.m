function tilefigs()
% <cpp> tile figure windows


%Charles Plum                    Nichols Research Corp.
%<cplum@nichols.com>             70 Westview Street
%Tel: (781) 862-9400             Kilnbrook IV
%Fax: (781) 862-9485             Lexington, MA 02173

% Deepak Aswani	modified	2 Jan 2002


% constants
left_window_margin = 150;
side_width = 10;
top_space = 70;
bottom_space = 5;
start_height = 30;

maxpos  = get (0,'screensize'); % determine terminal size in pixels
hands   = get (0,'Children');   % locate all open figure handles
hands   = sort(hands);          % sort figure handles
numfigs = size(hands,1);        % number of open figures
maxfigs = fix(maxpos(4)/20);

if (numfigs>maxfigs)            % figure limit check
        disp([' More than ' num2str(maxfigs) ' requested '])
        return
end

stop = 0;
for col = 1:10
   for row = 1:col
      xdim = floor((maxpos(3) - left_window_margin)/col) - side_width;
      ydim = floor((maxpos(4) - start_height)/row) - top_space - bottom_space;
      if ((row*col) >= numfigs)
         stop = 1;
         break;
      end;
   end;
   if stop
      break
   end;
end;

% tile figures by postiion 
% Location (1,1) is at bottom left corner
%pnum=0;
%for iy = 1:numfigs
%  figure(hands(iy))
%  p = get(hands(iy),'Position'); % get figure position
%  
%  ypos = maxpos(4) - (iy-1)*20 -p(4) -45 ; % figure location (row)
%  ypos = maxpos(4) - (iy-1)*20 -p(4) -45 ; % figure location (row)
%  xpos = fix((iy-1)*5 + 15);     % figure location (column)
  
%  set(hands(iy),'Position',[ xpos ypos p(3) p(4) ]); % move figure
%end

fnum=0;
for icol = 1:col
   for irow = 1:row
      ypos = (irow - 1) * (ydim + top_space + bottom_space) + start_height + bottom_space;		% figure location (row)
      xpos = (icol - 1) * (xdim + side_width) + floor(side_width/2) + left_window_margin;					% figure location (column)
      fnum = fnum + 1;
      if fnum <= numfigs
      		set(hands(fnum),'Position', [xpos ypos xdim ydim]);            
      end;
   end;
end;

     
return;