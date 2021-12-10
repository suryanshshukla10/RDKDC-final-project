function mu = manipulability(J,measure)
% Author - Suryansh Shukla 
%Summary - Compute a measure of manipulability
% Input1 - J: a 6x6 matrix 
% Input2 - measure : string with 'signmamin', 'detjac',invcond'

% output - measure of manipulability

% Manipulability as in chapter 3 , section 4.4 

% Example 
% input1 J:eye(6)
% input2 measure:'signmamin'   
% output -> mu = 20

% % % % % % % % % % % % % % % % % % % % % % % % % % % 
%% Error messages 
assert(isa(measure,'char'),"Measure not in string")
assert(isa(J,'double'),"J not double")
[Jrow,Jcol] = size(J);
assert(Jrow==6,"Body Jacobian row is not equal to 6 ")
assert(Jcol==6,"Body Jacobian col is not equal to 6 ")

%% Singularity Calculation 
A = transpose(J)*J;% calculate J^T J 
[V,D,W] = eig(A); % Calc. eigen values of A
eigen_value = diag(D); %D: diag. matrix  of eigenvalues
sigma = sqrt(eigen_value); %Calc. square root of eigenvalues of A 
sigma_min = min(sigma); %Min Singular value of J
sigma_max = max(sigma); %Max Singular value of J

%% Minimum Singular Value J
if measure =="signmamin"
    mu = sigma_min;
    disp("Sigmamin")   
%% Determinant of J
elseif measure == "detjac"
        disp("Determinanat of J")
        mu = det(J);
%% Inverse condition number of J 
elseif measure == "invcond"
        disp("Inverse condition number of J")
        mu = sigma_min/sigma_max; 
end
end

