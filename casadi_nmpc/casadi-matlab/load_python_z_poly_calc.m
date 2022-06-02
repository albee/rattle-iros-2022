%{
Load the z_poly_calc Python module using MATLAB's interface.
%}
function [] = load_python_z_poly_calc()
  clear classes;  % needed to reload Python changes

  % Ensure module is on PYTHONPATH, force reload. Needs >=Python3.5!
  PYTHON_MODULE_PATH = '/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/z_poly_calc';
  P = py.sys.path;
  if count(P, PYTHON_MODULE_PATH) == 0
      insert(P,int32(0), PYTHON_MODULE_PATH);
  end
  z_poly_calc = py.importlib.import_module('z_poly_calc');
  py.importlib.reload(z_poly_calc);
end