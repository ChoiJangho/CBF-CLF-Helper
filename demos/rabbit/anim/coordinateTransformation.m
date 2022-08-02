function s_quan_vec = coordinateTransformation(s_vec)
% ours to quans
qrel = s_vec(:, 3:7)';
dqrel = s_vec(:, 10:14)';

T = [-1   -1   0  0  0;
     -1    0   0 -1  0;
     -1   -1  -1  0  0;
	 -1    0   0 -1 -1;
     -1    0   0  0  0];
 
q = T*qrel + 2*pi*[ones(4,1);0];
dq = T*dqrel;
s_quan_vec = [q; dq]';
end