%% Function to evaluate the orientation of the end effector
function [DRAO] = distOrientation(r,h)
    ar = r(1);br=r(2);cr=r(3);dr=r(4);
    ah = h(1);bh=h(2);ch=h(3);dh=h(4);
    drao_p = acos((ar*ah)+(br*bh)+(cr*ch)+(dr*dh));
    drao_n = acos((ar*-ah)+(br*-bh)+(cr*-ch)+(dr*-dh));
    DRAO = min(drao_p,drao_n); % Search of the antipodals points
    
end

