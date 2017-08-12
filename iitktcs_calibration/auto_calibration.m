camera_link=[-0.0692541 -0.297744 0.846851;
0.0469655 -0.287963 0.847811;
0.140122 -0.279689 0.844477;
0.254462 -0.270259 0.841001;
0.246267 -0.122051 0.846519;
0.133332 -0.144057 0.848616;
0.0419056 -0.158483 0.852056;
-0.0718632 -0.160725 0.853698;
-0.0793264 -0.0491366 0.859209;
0.0360702 -0.045173 0.856671;
0.129996 -0.0344494 0.853364;
0.228865 -0.0253356 0.850541;
0.237121 0.0777564 0.85513;
0.125348 0.0834622 0.859779;
0.0318796 0.0810296 0.8616;
-0.0883876 0.0739412 0.866067;];

hce = [0.0903809 0.190623 0.977494 0.0432301;
       -0.690086 -0.695695 0.199475 0.115773;
       0.718062 -0.692583 0.0686687 0.122649;
       0 0 0 1];

hec = [0.0903809 -0.690086 0.718062 -0.0120834;
       0.190623 -0.695695 -0.692583 0.157247;
       0.977494 0.199475 0.0686687 -0.0737732;
       0 0 0 1];

% eelink to world
H1 = [0.360678 -0.923855 0.128072 -0.140081;
      0.288022 -0.0202784 -0.957409 0.0776801;
      0.887105 0.382204 0.258777 1.40591;
       0 0 0 1];
% eelink to world
%H2 = [0.395659 -0.839926 0.371454 -0.135367
%      0.545558 -0.110411 -0.830768 0.0785843
%      0.738796 0.531351 0.414544 1.41714;
%       0 0 0 1];
% world to eelink
H2 = [0.395651 0.545438 0.738889 -1.03636;
      -0.839955 -0.110446 0.531298 -0.857971;
      0.371398 -0.830842 0.414445 -0.471797;
       0 0 0 1];

P_c1 = [camera_link ones(16,1)]';
P_c2 = hec*H2*H1*hce*P_c1;

P_c1 = P_c1(1:3,:);
P_c2 = P_c2(1:3,:);

%P_c1 = P_c1';
%P_c2 = P_c2';
N = size(P_c1,2);

R1 = H1(1:3, 1:3);
R2 = H2(1:3, 1:3);
M_PI = 3.14159265359;

error = 1000000000000000000;
angles(1:3,1) = 0;
Rce = [1 0 0; 0 1 0; 0 0 1];
for i=-180:1:180
	for j=-180:1:180
		for k=-180:1:180
			r=i*M_PI/180;
			p=j*M_PI/180;
			y=k*M_PI/180;
			Rr=[1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
			Rp=[cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
			Ry=[cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];

			Rce = Ry*Rp*Rr;
			lh = R2*R1*Rce*P_c1;
			rh = Rce*P_c2;

			e = sum(sqrt(sum((lh-rh).^2)));
			e = e/N;

			if(e<error)
				error = e;
				angles = [r p y];
			end
		end
	end
end
%disp('5 steps completed');
%disp(error);
%for i=angles(1)-5:1:angles(1)+5
%	for j=angles(2)-5:1:angles(2)+5
%		for k=angles(3)-5:1:angles(3)+5
%			r=i*M_PI/180;
%			p=j*M_PI/180;
%			y=k*M_PI/180;
%			Rr=[1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
%			Rp=[cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
%			Ry=[cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
%
%			Rce = Ry*Rp*Rr;
%			lh = R2*R1*Rce*P_c1;
%			rh = Rce*P_c2;
%
%			e = sum(sqrt(sum((lh-rh).^2)));
%			e = e/N;
%
%			if(e<error)
%				error = e;
%				angles = [r p y];
%			end
%		end
%	end
%end
disp('1 steps completed');
disp(error);
for i=angles(1)-1:0.1:angles(1)+1
	for j=angles(2)-1:0.1:angles(2)+1
		for k=angles(3)-1:0.1:angles(3)+1
			r=i*M_PI/180;
			p=j*M_PI/180;
			y=k*M_PI/180;
			Rr=[1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
			Rp=[cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
			Ry=[cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];

			Rce = Ry*Rp*Rr;
			lh = R2*R1*Rce*P_c1;
			rh = Rce*P_c2;

			e = sum(sqrt(sum((lh-rh).^2)));
			e = e/N;

			if(e<error)
				error = e;
				angles = [r p y];
			end
		end
	end
end
disp('0.1 steps completed');
disp(error);				
for i=angles(1)-0.1:0.01:angles(1)+0.1
	for j=angles(2)-0.1:0.01:angles(2)+0.1
		for k=angles(3)-0.1:0.01:angles(3)+0.1
			r=i*M_PI/180;
			p=j*M_PI/180;
			y=k*M_PI/180;
			Rr=[1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
			Rp=[cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
			Ry=[cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];

			Rce = Ry*Rp*Rr;
			lh = R2*R1*Rce*P_c1;
			rh = Rce*P_c2;

			e = sum(sqrt(sum((lh-rh).^2)));
			e = e/N;

			if(e<error)
				error = e;
				angles = [r p y];
			end
		end
	end
end

disp(error);
disp(Rce);
						
