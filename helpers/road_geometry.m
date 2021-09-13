% Backup code, in case it needs to be regenerated
lw = 3.0;
far = 25.0;
edge_SEvx =  lw*ones(ii,1);
edge_SEvy = linspace(-far,-lw,ii);
edge_SEhx = linspace(lw,far,ii);
edge_SEhy = -lw*ones(ii,1);

edge_SWvx = -lw*ones(ii,1);
edge_SWvy = linspace(-far,-lw,ii);
edge_SWhx = linspace(-far,-lw,ii);
edge_SWhy = -lw*ones(ii,1);

edge_NEvx =  lw*ones(ii,1);
edge_NEvy = linspace(lw,far,ii);
edge_NEhx = linspace(lw,far,ii);
edge_NEhy =  lw*ones(ii,1);

edge_NWvx = -lw*ones(ii,1);
edge_NWvy = linspace(lw,far,ii);
edge_NWhx = linspace(-far,-lw,ii);
edge_NWhy =  lw*ones(ii,1);

center_Sx =  0.0*ones(ii,1);
center_Sy = linspace(-far,-lw,ii);

center_Nx =  0.0*ones(ii,1);
center_Ny = linspace(lw,far,ii);

center_Ex = linspace(lw,far,ii);
center_Ey = 0.0*ones(ii,1);

center_Wx = linspace(-far,-lw,ii);
center_Wy = 0.0*ones(ii,1);

obstacles = [struct('x',edge_SEvx,'y',edge_SEvy,'color','k'),
             struct('x',edge_SEhx,'y',edge_SEhy,'color','k'),
             struct('x',edge_SWvx,'y',edge_SWvy,'color','k'),
             struct('x',edge_SWhx,'y',edge_SWhy,'color','k'),
             struct('x',edge_NEvx,'y',edge_NEvy,'color','k'),
             struct('x',edge_NEhx,'y',edge_NEhy,'color','k'),
             struct('x',edge_NWvx,'y',edge_NWvy,'color','k'),
             struct('x',edge_NWhx,'y',edge_NWhy,'color','k'),
             struct('x',center_Sx,'y',center_Sy,'color','y'),
             struct('x',center_Nx,'y',center_Ny,'color','y'),
             struct('x',center_Ex,'y',center_Ey,'color','y'),
             struct('x',center_Wx,'y',center_Wy,'color','y')];