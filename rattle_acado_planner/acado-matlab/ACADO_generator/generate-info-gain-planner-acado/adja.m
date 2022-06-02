function [adja] = adja(I)
  %{
  adjugate matrix of a 3x3, explicitly defined for ACADO
  %}
  adja = [ I(2,2)*I(3,3) - I(2,3)*I(3,2), I(1,3)*I(3,2) - I(1,2)*I(3,3), I(1,2)*I(2,3) - I(1,3)*I(2,2);
           I(2,3)*I(3,1) - I(2,1)*I(3,3), I(1,1)*I(3,3) - I(1,3)*I(3,1), I(1,3)*I(2,1) - I(1,1)*I(2,3);
           I(2,1)*I(3,2) - I(2,2)*I(3,1), I(1,2)*I(3,1) - I(1,1)*I(3,2), I(1,1)*I(2,2) - I(1,2)*I(2,1)];
end