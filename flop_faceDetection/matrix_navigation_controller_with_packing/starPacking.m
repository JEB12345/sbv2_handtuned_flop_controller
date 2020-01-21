SBtransform = [
    19 20 21 15 8 2 14 9 4 13 7 6 1 18 12 3 17 11 5 16 10 23 24 22;
    18 3 13 22 4 15 7 14 20 9 17 1 21 2 11 8 16 24 10 19 5 12 23 6 
    ];

slope = [-1.13272388059701,-1.14042158304600,-1.17338351598342,-1.06945782765140,-1.11349186621372,-1.14497930796643,-1.12547082818874,-1.08895423738307,-1.08169633323703,-1.12573961622541,-1.12234550720277,-1.12037814814265,-1.11246037551817,-1.11100024243662,-1.10440750557301,-1.13555410889649,-1.11643316697733,-1.09564556365700,-1.12041645665323,-1.13072891187365,-1.14080189055838,-1.17501817049112,-1.13024261777840,-1.10180013546606];
offset = [108.254000000000,100.192000000000,124.433000000000,101.073000000000,110.140000000000,114.641000000000,108.872000000000,109.739000000000,104.905000000000,111.072000000000,113.721000000000,106.830000000000,114.410000000000,108.207000000000,94.9220000000000,114.409000000000,109.408000000000,110.794000000000,107.684000000000,104.415000000000,105.778000000000,105.691000000000,109.936000000000,106.484000000000] + 5 ;

[v,i_star]=sort(SBtransform(1,:));
SBtransformSort = SBtransform(:,i_star);

unpackingLenTen = csvread('hex_pack_len_ten_prestress.csv');

Group.setCommandLifetime(0);
Cmd.position = ones(1,24)*NaN;

Cmd.position = ones(1,24)*0.0;
Group.send(Cmd);

disp('Press any key to pack down...')
pause()


%% Loop
for i_star=[ones(1,1)*size(unpackingLenTen,1), (size(unpackingLenTen,1)):-1:100]
    current_lengths = unpackingLenTen(i_star,1:24);
    current_tensions = unpackingLenTen(i_star,25:end);
    
    current_tensions(current_tensions > 400.0) = 400.0;
    current_moments = current_tensions*0.006;
    current_restLengths = current_lengths;
    
    newRestLengths(SBtransformSort(2,:)) = current_restLengths;
    newMoments(SBtransformSort(2,:)) = current_moments;
    newMoments;
        
    cmdMotorPositions = (100*newRestLengths).*slope + offset;
    %pause
    %% While Loop
    for stupid=1:1
        Cmd.position = cmdMotorPositions;
        Cmd.effort = newMoments*0.2;
        Group.send(Cmd);
        %pause(0.025)
        pause(0.05)
    end

end

disp('DOWN! Press any key to redeploy');
pause();

%%


Group.setCommandLifetime(0);
Cmd.position = ones(1,24)*NaN;

Cmd.position = ones(1,24)*0.0;
Group.send(Cmd);