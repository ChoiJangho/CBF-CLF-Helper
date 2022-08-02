function out = impact_abs_red(u)
%
% From Jessy's code simulator.
%
q = u(1:5);
dq = u(6:10);

[g,L1,L3,L4,M1,M3,M4,MY1,MZ1,MZ3,MZ4,XX1,XX3,XX4] = modelParameters;

q31 = q(1);    q32 = q(2);   q41 = q(3);   q42 = q(4);   q1 = q(5);
dq31 = dq(1); dq32 = dq(2); dq41 = dq(3); dq42 = dq(4); dq1 = dq(5);

pH = [L3*sin(q31)+L4*sin(q41);
      -L3*cos(q31)-L4*cos(q41)];
vH = [L3*cos(q31)*dq31+L4*cos(q41)*dq41;
      L3*sin(q31)*dq31+L4*sin(q41)*dq41];

[num_r,num_c] = size(q);
if num_r > num_c
  qe=[q(1:4);pH;q(5)];
  dq_minus=[dq(1:4);vH;dq(5)];
else
  qe=[q(1:4).';pH;q(5)];
  dq_minus=[dq(1:4).';vH;dq(5)];
end

[De,Ce,Ge,Be,Ee] = dyn_mod_abs([qe;dq_minus]);

E = Ee(:,3:4);
% De*(dq^+ - dq^-) = E*impulsive forces=E*Fe
% E'*dq^+ = 0;
%
A = [De,-E;E',zeros(2,2)];
B = [De*dq_minus;zeros(2,1)];
%cond_impact = cond(A)
ans = inv(A)*B;
dq31 = ans(1); dq41 = ans(3); dz = ans(6);
vFoot_1 = -L3*sin(q31)*dq31-L4*sin(q41)*dq41+dz; % to test that after impact 
                                                 % the fixed foot raises
                                                 % itself from the ground
Fe = ans(8:9);
test1 = min(100,abs(Fe(1)/Fe(2)));
test2 = vFoot_1;
if (test1 <= 2/3 & test2 >= 0)
   flag = 0;
   q_plus_new = [q(2);q(1);q(4);q(3);q(5)];
   dq_plus_old_coord = [ans(1:4);ans(7)];
   dq_plus_new = [dq_plus_old_coord(2);dq_plus_old_coord(1); ...
     dq_plus_old_coord(4); dq_plus_old_coord(3);dq_plus_old_coord(5)];
   out = [q_plus_new;dq_plus_new;flag;test1;test2];
elseif (test2 < 0)
   flag = 1;

   disp('[IMPACT_ABS_RED.M : need abs_red_2]'); %pause HERE

   if 0 % just for debugging's sake
     q_plus_new = [q(2);q(1);q(4);q(3);q(5)];
     dq_plus_old_coord = [ans(1:4);ans(7)];
     dq_plus_new = [dq_plus_old_coord(2);dq_plus_old_coord(1); ...
		    dq_plus_old_coord(4); dq_plus_old_coord(3); ...
		    dq_plus_old_coord(5)];
     out = [q_plus_new;dq_plus_new;flag;test1;test2]
   end
     
   out = zeros(13,1); %impact_abs_red_2([q;dq]); HERE
else
   flag = 1;

   q_plus_new = [q(2);q(1);q(4);q(3);q(5)];
   dq_plus_old_coord = [ans(1:4);ans(7)];
   dq_plus_new = [dq_plus_old_coord(2);dq_plus_old_coord(1); ...
     dq_plus_old_coord(4); dq_plus_old_coord(3);dq_plus_old_coord(5)];
   out = [q_plus_new;dq_plus_new;flag;test1;test2];
end