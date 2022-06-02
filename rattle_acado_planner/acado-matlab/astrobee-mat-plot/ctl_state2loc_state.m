function x_loc = ctl_state2loc_state(x)
  %{
  Converts the 13x1 localization state vector to the
  20x1 control state vector

  x, [20 x n]: [t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd]'
  x_loc, [13 x n]: [x y z xd yd zd qx qy qz qw wx wy wz]'
  %}
  x_loc = x(2:14, 1:end);
end