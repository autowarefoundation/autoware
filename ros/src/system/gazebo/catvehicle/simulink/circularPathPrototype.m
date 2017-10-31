%% my test file
% To visualize the angles, run this code:
radius=3.6; start=[0 0];
angles=linspace(-pi,pi,31);
r = radius*exp(angles*1j);
xs = real(r)-radius+start(1);
ys = imag(r)+start(2);
% since the direction of travel is in the 'x' direction, the 
% heading (phi) matchines the angle of the circle
phis=(pi/2)+angles;
xends=cos(phis);yends=sin(phis);
figure;plot(xs,ys);hold on; quiver(xs,ys,xends,yends);axis equal;hold off