%% Check if the configuration generates a position out of the limits
PuntoEnd = getTransform(robotModel,goalConfig','tool0','world');
dif = norm([PuntoEnd(1,4) PuntoEnd(2,4) PuntoEnd(3,4)] - pointEnd);

if dif > 0.05
    correct = false;
else
    correct = true;
end