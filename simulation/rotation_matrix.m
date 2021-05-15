syms R p q u

R = [cos(q)*cos(u), sin(p)*sin(q)*cos(u)-cos(p)*sin(u), cos(p)*sin(q)*cos(u)+sin(p)*sin(u) ;
     cos(q)*sin(u), sin(p)*sin(q)*sin(u)+cos(p)*cos(u), cos(p)*sin(q)*sin(u)-sin(p)*cos(u) ;
     -sin(q),       sin(p)*cos(q),                      cos(p)*cos(q)                     ];

Rinv = R.'; 
 
 p = 0;
 q = 0;
 
subs(Rinv)