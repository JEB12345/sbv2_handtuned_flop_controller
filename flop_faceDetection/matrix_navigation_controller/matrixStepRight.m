% Function to move forward on the same ring
function f=matrixStepRight(j,COLUMNS)
    f=mod(j,COLUMNS)+1;
end
