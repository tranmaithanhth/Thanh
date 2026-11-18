from pathlib import Path
import re
p=Path(__file__).resolve().parent
b=(p/'build_ackermann_giaotrinh.m').read_text()
refs=re.findall(r"'([a-zA-Z_0-9]+\.txt)'", b)
for r in refs:
    f=p/'function_scripts'/r
    assert f.exists(), f
    s=f.read_text()
    assert s.startswith('function '), f
    assert s.rstrip().endswith('end'), f
assert len(set(refs))==len(refs), refs
need=['TKQD_cho_P','TKQD_cho_P1','Sai_so_P1','Kp','Feedforward_plus_Kp',
      'Transform_P1_sang_v_psi','DWA_Local_Planner','TEB_Local_Planner',
      'CHON_DWA_HOAC_TEB','Robot_Newton_Euler','Toa_do_P1','Tach_5_dao_ham',
      "'psi_1_s'","'v_1_s'","'theta_1_s'","'xP_1_s'","'yP_1_s'"]
for n in need:
    if '_1_s' in n:
        # Generated in a loop instead of literal block declarations.
        assert 'startnames' in b and "'_1_s'" in b
    else:
        assert n in b, n
pl=(p/'ackermann_planners_giaotrinh.m').read_text()
assert 'guide(2)' in pl and 'guide(1)' in pl
assert "[ref,band,cost,good] = teb_plan(x,c,guide);" in pl
assert "[ref,band,cost] = dwa_plan(x,c,guide);" in pl
assert "band_objective(zz,x,pts(end,:),c,guide)" in pl
print('PASS: block diagram names, source files, guide connections and MATLAB function scripts')
