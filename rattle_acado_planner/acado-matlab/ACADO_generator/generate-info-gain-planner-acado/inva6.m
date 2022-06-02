function [inv_M] = inva6(M)
  %{
  Block inversion of a 6x6 square matrix. This breaks M broken into 3x3 chunks.
  %}
  M11 = M(1:3, 1:3);
  M12 = M(1:3, 4:6);
  M21 = M(4:6, 1:3);
  M22 = M(4:6, 4:6);

	inv_M11 = inva(M11);
         
  % trick to invert a block matrix
  special_inv = inva(M22 - M21*inv_M11*M12);
  
  A11 = inv_M11 + inv_M11*M12*(special_inv)*M21*inv_M11;
  A12 = -inv_M11*M12*special_inv;
  A21 = -special_inv*M21*inv_M11;
  A22 = special_inv;
  
  inv_M = [A11, A12;
           A21, A22];
end