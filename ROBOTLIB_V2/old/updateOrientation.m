function r_current = updateOrientation(r_current, w_current, time_step)
    skew = @(v) [   0    -v(3)   v(2); 
                  v(3)    0    -v(1); 
                 -v(2)   v(1)    0  ];
    r_current = r_current * expm(skew(w_current * time_step));
end