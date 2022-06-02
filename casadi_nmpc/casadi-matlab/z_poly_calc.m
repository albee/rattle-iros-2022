%{
 Script to compute a polytope tube for a linear system with additive disturbances
 Uses Multi-Parametric Toolbox for polytope set calculations https://www.mpt3.org/
 Charles Oestreich, Caroline Specht, Keenan Albee
 Adapted to Python for TumbleDock use.
%}

function [Kdr, Z, Utight] = z_poly_calc(A_c, B_c, Q_LQR, R_LQR, W_bounds, U_max, DT)
  plotting = 0;  % can't plot above 4d

  % parameters
  n = size(A_c, 1);
  m = size(B_c, 2);
  Ts = DT; % sample time
  
  Q_dr = Q_LQR;
  R_dr = R_LQR;

  epsilon = 0.05;  % mRPI calculation parameter (sort of like an error threshold)
   

  C_c = eye(n);
  D_c = zeros(n, m);
  sys_c = ss(A_c,B_c,C_c,D_c);

  % discrete conversion using zero-order hold
  sys_d = c2d(sys_c, Ts);
  A = sys_d.A
  B = sys_d.B
  C = sys_d.C;
  D = sys_d.D;

  % example uncertainty bounds (polytope):
%   verts = [-0.05, -0.01;
%            0.05, -0.01;
%            -0.05, 0.01;
%            0.05, 0.01];
%   W = Polyhedron('V', verts, 'R', []);
  
  % disturbance
  Aw = [eye(n);
     -eye(n)];
  bw = [W_bounds;
      W_bounds];
  W = Polyhedron(Aw, bw);

  % actuation constraints
  Au = [eye(m);
       -eye(m)];
  bu = [U_max;
        U_max];
  U = Polyhedron(Au, bu);
  

  % calculate disturbance rejection gains via LQR
  [Kdr, Pdr, ~] = dlqr(A, B, Q_dr, R_dr);
  Kdr = -Kdr;
  Acl = A + B*Kdr;
  disp('LQR gain calculated...');

  %% mRPI algorithm
  % Reference: Algorithm (1) from Rakovic et al.
  % Assumes W is a polytope

  tic

  % Initialize s
  s = 0;

  % Initialize alpha in (0, 1)
  alpha = 0.99;

  % Calculate the first iteration of M(s)
  Mplus = zeros(n,1);
  Mminus = zeros(n,1);
  basis = eye(n);
  
  % iterate over dimensions of W
  for i = 1:n
    Mplus(i) = Mplus(i) + W.support((Acl^s)'*basis(:,n));
    Mminus(i) = Mminus(i) + W.support(-(Acl^s)'*basis(:,n));
  end
  
  % get maximum over all dimensions and plus/minus
  Msum = max(max(Mplus), max(Mminus));

  % update s
  s = s + 1;
  
  
  % Repeate to minimize alpha below the error bound
  while alpha > (epsilon / (epsilon + Msum))
      % calculate new alpha
      supp_iter = zeros(1,length(W.b));
      for i = 1:length(W.b)
          supp_iter(i) = W.support((Acl^s)'*W.A(i,:)') / W.b(i);
      end
      alpha = max(supp_iter);

      % Calculate error bound again
      for i = 1:n
          Mplus(i) = Mplus(i) + W.support((Acl^s)'*basis(:,n));
          Mminus(i) = Mminus(i) + W.support(-(Acl^s)'*basis(:,n));
      end
      Msum = max(max(Mplus), max(Mminus));

      % update s for next iteration
      s = s + 1;
  end

  % Minkowski sums for Fs
  if (plotting == 1)
    figure(1);
  end
  
  Fs = W;
  s
  for i = 1:1  % (s-1)
    i
      Fs = (Acl^i)*W + Fs;
      % Show growth of Z approximation
      if (plotting == 1)
        plot(Fs, 'color', 'black', 'wire', 1);
        hold on;
      end
  end

  % Scale using alpha for Z
  Z = (1 - alpha)^(-1) * Fs;

  toc
  
  KZ = mtimes(Kdr,Z);
  Utight = minus(U,KZ);
  
  % Plot Z polytope approximtion of mRPI
  if (plotting == 1)
      figure(2);
      plot(Z, 'color', 'black', 'wire', 1);

      xlabel('[m]');
      ylabel('[m/s]');
      title('$\mathcal{Z}$ (approx. mRPI)');
      grid on;
  end
end
    

