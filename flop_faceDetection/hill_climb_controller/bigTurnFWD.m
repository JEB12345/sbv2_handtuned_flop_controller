function [newi,newj]=bigTurnFWD(M,currentFace,compareTerm,dir)
   for l=1:4
        for k=1:6
            if M(l,k)==currentFace && dir==0 && M(l,matrixStepLeft(k,6))==compareTerm
                newi=l;
                newj=matrixStepRight(k,6);
                return
            elseif M(l,k)==currentFace && dir==1 && M(l,matrixStepRight(k,6))==compareTerm
                newi=l;
                newj=matrixStepLeft(k,6);
                return
            end
        end
    end 
end