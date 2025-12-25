function [Hq] = FunctionCalculateHq(Twists,H_i0,q)
%UNTITLED11 Compute the forward position of the end effector in terms of
%the angles.
    Hq = 1;
    ind = 1;
    while ind<=length(q)  
        Hq = Hq*expm(Twists(:,(4*(ind-1)+1):4*(ind))*q(ind));
        ind = ind+1;
    end 
    Hq = Hq*H_i0;
end