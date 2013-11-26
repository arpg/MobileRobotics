function [v]= NearN(A,i,j)

[n,m]=size(A);

if (i==1 & j==1)
    v=[[2;1],[1;2]];  
else if (i==n & j==m)
        v=[[n;m-1],[n-1;m]];
else if (i==1 & j==m)
        v=[[1;m-1],[2;m]];
else if (i==n & j==1)
        v=[[n-1;1],[n;2]];
    else if(j==1)
        v=[[i+1;1],[i;j+1],[i-1;1]] ;
    else if(i==1)
        v=[[i;j+1],[i;j-1],[i+1;j]] ;
    else if(j==m)
        v=[[i+1;j],[i-1;j],[i;j-1]] ;
    else if(i==n)
        v=[[i;j+1],[i;j-1],[i-1;j]] ;
        else 
        v=[[i;j+1],[i;j-1],[i-1;j],[i+1;j]] ;
            end
        
        end 
        end    
        end
    
    
    end
    end    
    end
end


end
