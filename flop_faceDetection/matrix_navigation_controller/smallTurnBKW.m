function [newi,newj]=smallTurnBKW(M,currentFace,compareTerm,dir)
    for l=1:4
        for k=1:6
            if M(l,k)==currentFace && dir==0 && M(l,backwards(k,6))==compareTerm
                newi=l;
                newj=backwards(k,6);
                return
            elseif M(l,k)==currentFace && dir==1 && M(l,forward(k,6))==compareTerm
                newi=l;
                newj=forward(k,6);
                return
            end
        end
    end
end