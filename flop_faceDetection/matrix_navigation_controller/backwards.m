% Funciton to move backwards on the same ring
function f=backwards(j,COLUMNS)
    f= mod(j+ COLUMNS-2,COLUMNS)+1;
end
