%% Fucntion to evaluate the position of the end effector
function [D_rax] = distPosition(A,B)
    D_rax =abs(norm(A-B));
end

