function [w_tilde] = FunctionSkew(w)
    %UNTITLED10 Converts xyz coordenates to skewsimmetric
    w_tilde = [ 0    -w(3)  w(2);
          w(3)  0    -w(1);
         -w(2) w(1)   0];
end