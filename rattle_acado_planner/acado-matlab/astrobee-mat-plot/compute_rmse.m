%{
Computes the (normalized) discrete root mean square.

Inputs
x_vec_ref: reference vector [m, n], x is row
x_vec: actual x vector [m, n], x is row
normalized: True or False

[x1 x2 x3;
 x1 x2 x3;
 ...
]

Outputs:
RMS: root mean square error for the history
%}
function [RMS, vec_RMS] = calc_RMS_error(x_vec_ref, x_vec, normalized)
    x_err = x_vec_ref - x_vec;
    RMS = 0;
    m = size(x_err,1);
    n = size(x_err,2);
    
    if normalized  % normalized for each state's max and min
        % TODO
    end
    
    for i = 1:m
        for j = 1:n
            x = x_err(i, j);
            RMS = RMS + x^2;
        end
    end
    RMS = sqrt(RMS/m);
    
    
    vec_RMS = sqrt(sum(x_err.*x_err)/length(x_err));
end