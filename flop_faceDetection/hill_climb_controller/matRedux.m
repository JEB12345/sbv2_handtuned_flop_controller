% Function that can be used for optimization (iterate on a smaller matrix)
function Red = matRedux(M,faceMat,i,j)
    reduxIndex = M(i,j);
    count=1;
    for k=1:3
        oldIndex= faceMat(reduxIndex,k);
        if oldIndex~=i
            Red(count,:)=M(oldIndex,:);
            count =count+1;
        end
    end
end