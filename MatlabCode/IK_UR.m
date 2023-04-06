%% Analytical inverse kinematics for the UR arms

M = matrix;
name = "ur3"; % Model of the robotic arm for the IK calculus
launcher = "matlab"; % To indicate that has been launch by matlab
try
    npM = py.numpy.array(M);
catch

end
% Convert to numpy.array values for python
npM = py.numpy.array(M(:).');
res = double(pyrunfile("UR3_Inverse_Kinematics.py","Sol",M = npM,name = name,launcher = launcher));


RES = [RES;res];