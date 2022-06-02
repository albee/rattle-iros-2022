function ocp = create_ocp(N, dt, S, h, SN, hN, u, f)
  %{
  Create an ACADO OCP function
  %}
  ocp = acado.OCP(0.0, N*dt, N);  % start time, end time, number of intervals: OCP(tStart, tEnd, N)

  % cost function
  ocp.minimizeLSQ(S, h);  
  ocp.minimizeLSQEndTerm(SN, hN);

  % constraints
  ocp.subjectTo(-0.15 <= u(1:3) <= 0.15);  % control constraints
  ocp.subjectTo(-0.015 <= u(4:6) <= 0.015);  % control constraints
  ocp.setModel(f);  % constraint from dynamic model
end