%{
Computes the (normalized) discrete root mean square.

Inputs
x_vec_ref: reference vector [m, n], x is row
x_vec: actual x vector [m, n], x is row
normalized: {True, False}
sync: {True, False}. If True, finds closest points between ref and real.

[x1 x2 x3;
 x1 x2 x3;
 ...
]

Outputs:
RMS: total root mean square error for the history
vec_RMS: error per element (?)
%}
function [RMS, vec_RMS] = calc_RMS_error(t_start, t_end, t_x_des, t_x_real, x_vec_ref, x_vec, normalized, sync, type)    
    % (1) Normalize max/min values
    if normalized  % normalized for each state's max and min
        % TODO
    end
    
    % (2) Sync up time stamps to closest possible 
    idxs = [];
    if sync
      for i = 1:1:length(t_x_des)  % for all desired times...
        [~, idx] = min(abs(t_x_real - t_x_des(i)));  % find the closest recorded time index...
        idxs(end+1) = idx;
      end
    end
    t_x_real = t_x_real(idxs);
    x_vec = x_vec(idxs, :);
    
    % (3) Get start and stop timestamps
    [~, idx_start] = min(abs(t_start - t_x_des))
    [~, idx_end] = min(abs(t_end - t_x_des))
   
    % (4) Compute RMSE
    x_err = x_vec_ref - x_vec;
    RMS = 0;
    m = size(x_err,1);
    n = size(x_err,2);
    
    for i = idx_start:idx_end
      if type ~= "translation"
        for j = 1:n
            x = x_err(i, j);
            RMS = RMS + x^2;
        end
      elseif type == "translation"
        for j = 1:6
          x = x_err(i, j);
          RMS = RMS + x^2;
        end
      end
    end
    
    RMS = sqrt(RMS/m);
    
    vec_RMS = sqrt(sum(x_err.*x_err)/length(x_err));
end