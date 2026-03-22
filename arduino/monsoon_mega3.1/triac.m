xv = 0:0.01:1;
Ppv = 200/(4*pi)*(sin(2*pi*xv) - 2*pi*(xv-1));
plot(xv,Ppv);

Ppavh = @(x) 100/(2*pi)*(sin(2*pi*x) - 2*pi*(x-1));
dPpavh = @(x) 100/(2*pi)*(2*pi*cos(2*pi*x) - 2*pi);

Pp = 95;  % target
xc = 0.5;
for i=1:10
  fxc = Ppavh(xc) - Pp;
  fpxc = dPpavh(xc);
  xn = xc - fxc/fpxc;
  if abs(xc-xn)<1e-4, break; end
  xc = xn
end

Ppavh(xc)  % actual