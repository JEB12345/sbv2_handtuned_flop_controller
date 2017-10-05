% Funciton to move backwards on the same ring
function f=matrixStepLeft(j,COLUMNS)
    f= mod(j+ COLUMNS-2,COLUMNS)+1;
end
