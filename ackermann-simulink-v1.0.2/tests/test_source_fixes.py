from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
script = (ROOT / 'function_scripts' / 'tkqd_p.txt').read_text()
assert 'function [pd, pdot] = tkqd_p(t,x,route)' in script
assert script.index('q = route(n,:);') < script.index('for j=1:(n-1)')
assert 'route = [0.0' not in script
planner = (ROOT / 'msfcn_planner_giaotrinh.m').read_text()
assert "band0 = repmat(c.start(1:2).',1,c.pathNodes);" in planner
assert 'band0(:)' in planner
builder = (ROOT / 'build_ackermann_giaotrinh.m').read_text()
assert "set_param(mdl,'SimulationCommand','update');" in builder
assert "GlobalRoute_ASTAR" in builder
assert "'TKQD_cho_P',{'t','X','Route'}" in builder
assert "'GlobalRoute_ASTAR/1','TKQD_cho_P/3'" in builder
cfg = (ROOT / 'ackermann_cfg.m').read_text()
assert 'c.route = ackermann_global_route(c)' in cfg
assert 'c.route = [0.' not in cfg
assert (ROOT/'ackermann_global_route.m').exists()
print('PASS: A* global path is shared by TKQD and local planners; no hardcoded route')
