function [inva] = inva(I)
  %{
  Approximate inverse of a 3x3, explicitly defined for ACADO.
  Must be VERY careful for proper symbolic division!
  %}
  res1 = adja(I);
  res2 = deta(I);
  inva = simplify((res1)/(res2));
end