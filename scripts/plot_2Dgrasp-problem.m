#!/usr/bin/env octave-cli
% Author: Ugo Pattacini - <ugo.pattacini@iit.it>

function w=rotz(v,ang)
  w=[cos(ang) -sin(ang); sin(ang) cos(ang)]*[v(1); v(2)];
  w=reshape(w,size(v));
end

function display_friction_cone(F,friction)
  ang=atan(friction);
  p(1,:)=F(1:2);
  N=F(7:8) * .2/norm(F(7:8));
  p(2,:)=p(1,:)+rotz(N,ang);
  p(3,:)=p(1,:)+rotz(N,-ang);
  patch(p(:,1),p(:,2),'facecolor','cyan');
end

arg_list=argv();
filename=arg_list{1};

% load data
fid=fopen(filename,'r');
N=fscanf(fid,'%d',1);
p=zeros(N,3);
for i=1:N
  p(i,:)=fscanf(fid,'%f %f %f',[1 3]);
end
friction=fscanf(fid,'%f',1);
F0=fscanf(fid,'%f %f %f %f %f %f %f %f',[1 8]);
F1=fscanf(fid,'%f %f %f %f %f %f %f %f',[1 8]);
F2=fscanf(fid,'%f %f %f %f %f %f %f %f',[1 8]);
COM=fscanf(fid,'%f %f',[1 2]);
Ft=fscanf(fid,'%f %f',[1 2]);
Mt=fscanf(fid,'%f',1);
fclose(fid);

% plot data
figure('color','white');
hold('on'); grid('on');
xlabel('x'); ylabel('y');
set(gca,'layer','top');
axis('equal');
patch(p(:,2),p(:,3),'facecolor',[255 255 196]/255);
plot(COM(1),COM(2),'ro');
quiver(F0(1),F0(2),F0(3)+F0(5),F0(4)+F0(6),'k');
quiver(F1(1),F1(2),F1(3)+F1(5),F1(4)+F1(6),'r');
quiver(F2(1),F2(2),F2(3)+F2(5),F2(4)+F2(6),'r');
display_friction_cone(F0,friction);
display_friction_cone(F1,friction);
display_friction_cone(F2,friction);
text(1.05,.95,sprintf('F = (%.3f %.3f)',Ft(1),Ft(2)));
text(1.05,.85,sprintf('M = %.3f',Mt));

% save plot as file
out=[filename '.png'];
saveas(gcf,out);
