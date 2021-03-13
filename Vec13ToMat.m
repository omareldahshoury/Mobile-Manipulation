function [Matrix] = Vec13ToMat(X) 

Matrix = [X(1,1:3) X(1,10) ;X(1,4:6) X(1,11);X(1,7:9) X(1,12);0 0 0 1];

end