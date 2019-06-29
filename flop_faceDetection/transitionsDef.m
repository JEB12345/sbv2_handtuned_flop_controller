%% Motor Commands to transition from a base triangle to another one
% New definition: transition{origin, destination}
% The first value in [ ] is the main actuated cable on the base triangle.
% The other ones are extra cables that help complete the flop.

% Some definitions:

% Triangles     motors on the base triangle

% Triangle 1:   5   10  19  
% Triangle 2:   4   15  22
% Triangle 3:   13  18  3
% Triangle 4:   12  23  6
% Triangle 5:   21  2   11
% Triangle 6:   20  7   14
% Triangle 7:   8   16  24
% Triangle 8:   17  1   9


% Actuation scheme

% Initial triangle  Actuated cable  Destination triangle
%       1                   5               4
%       1                   10              8
%       1                   19              6
%       2                   4               3
%       2                   15              7
%       2                   22              5
%       3                   13              6
%       3                   18              8
%       3                   3               2
%       4                   12              5
%       4                   23              7
%       4                   6               1
%       5                   21              2
%       5                   2               8
%       5                   11              4
%       6                   20              1
%       6                   7               7
%       6                   14              5
%       7                   8               6
%       7                   16              2
%       7                   24              4
%       8                   17              3
%       8                   1               5
%       8                   9               1

transition = cell(8,8);

for ii = 1:8
    for jj = 1:8
        transition{ii, jj} = [NaN NaN NaN NaN NaN];
    end
end

clear ii jj;

transition{1,4} = [5    3   11  1   8];
transition{1,8} = [10   16  18  14 	11];
transition{1,6} = [19   21  8   23  18];

transition{2,3} = [4    6   14  8   1];
transition{2,7} = [15   9 	23 	11 	14];
transition{2,5} = [22   20 	1 	18 	23];

transition{3,6} = [13   11  19  9   16];
transition{3,8} = [18   24 	2 	22 	19];
transition{3,2} = [3    5 	16 	7 	2];

transition{4,5} = [12   14  22  16  9];
transition{4,7} = [23   17  7 	19  22];
transition{4,1} = [6    4   9 	2   7];

transition{5,2} = [21   19  3   17  24];
transition{5,8} = [2    8 	10 	6 	3];
transition{5,4} = [11   13  24 	15 	10];

transition{6,1} = [20   22  6   24  17];
transition{6,7} = [7    1 	15 	3 	6];
transition{6,3} = [14   12  17 	10 	15];

transition{7,6} = [8    2 	13 	4 	5];
transition{7,2} = [16   10 	21 	12 	13];
transition{7,4} = [24   18 	5 	20 	21];

transition{8,3} = [17   23 	4 	21 	20];
transition{8,5} = [1    7 	12 	5 	4];
transition{8,1} = [9    15 	20 	13 	12];

