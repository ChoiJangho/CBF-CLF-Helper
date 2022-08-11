function hPatch =drawShadedRectangle(vecXMinMax,vecYMinMax,vecC1,vecC2,vecC3,vecA1,vecA2,vecA3)
%MED 23 Feb 2005
%This will create shaded bars on a chart, or can be used to fill the
%background on a chart.
% e.g. drawShadedRectangle([0 1],[0 1],[],[],[],[],'horizontal')
% e.g. drawShadedRectangle([0 1],[0 1],[1 1 1],[0.5 0.5 0.5],[0 0 1],[0 0 0],'vertical'
%vecXMinMax are the x coordinates of the box
%vecYMinMax are the y coordinates of the box
%vecC1..vecC2 are rgb 1x3 colour vectors.


if ~exist('vecXMinMax','var'), vecXMinMax=[0 1];end;
if isempty(vecXMinMax),vecXMinMax=[0 1];end;
if ~exist('vecYMinMax','var'),vecYMinMax=[0 1];end;
if isempty(vecYMinMax),vecYMinMax=[0 1];end;
if ~exist('vecC1','var'),vecC1 =[0.95 0.95 0.95]/2;end;
if isempty(vecC1),vecC1 =[0.95 0.95 0.95]/2;end;
if ~exist('vecC2','var'),vecC2= vecC1*2;end;
if isempty(vecC2),vecC2= vecC1*2;end;
if ~exist('vecC3','var'),vecC3 =vecC1;end;
if isempty(vecC3),vecC3 =vecC1;end;
if ~exist('vecA1','var'),vecA1 =0.7;end;
if isempty(vecA1),vecA1 =0.7;end;
if ~exist('vecA2','var'),vecA2= 0.5;end;
if isempty(vecA2),vecA2= 0.5;end;
if ~exist('vecA3','var'),vecA3 = 0.0;end;
if isempty(vecA3),vecA3 = 0.0;end;
typeHorzVert = 'Horizontal';


vecV = [0 0;0 1;0 2;2 0;2 1;2 2]/2;
txtChck =lower(typeHorzVert);
switch txtChck
    case 'horizontal'    
    case 'vertical'
        vecV = fliplr(vecV);
    otherwise
     error('unrecognised typeHorzVert, must be horizontal or vertical');   
end    
        vecFaceOrder = [1 2 5 4 1;2 3 6 5 2];
        vecCData =[vecC1;vecC2;vecC3;vecC1;vecC2;vecC3];
        vecAData =[vecA1; vecA2; vecA3; vecA1; vecA2; vecA3];

vecV(:,1) = vecV(:,1)*(vecXMinMax(2)-vecXMinMax(1))+vecXMinMax(1);
vecV(:,2) = vecV(:,2)*(vecYMinMax(2)-vecYMinMax(1))+vecYMinMax(1);

        
tcolor = [0.75 0.5 0;0 0 0.5];
hPatch =patch('Faces',vecFaceOrder,'Vertices',vecV,'FaceVertexCData',tcolor,...
      'FaceColor','interp','FaceVertexCData',vecCData,...
      'FaceAlpha','interp','FaceVertexAlphaData',vecAData,'edgecolor','none');
  