function [deta] = deta (I)
  %{
  determinant of a 3x3, explicitly defined for ACADO
  %}
  deta =  I(1,1)*I(2,2)*I(3,3) + I(1,2)*I(2,3)*I(3,1) + I(1,3)*I(2,1)*I(3,2) ...
          -I(1,3)*I(2,2)*I(3,1) - I(1,2)*I(2,1)*I(3,3) - I(1,1)*I(2,3)*I(3,2);
end