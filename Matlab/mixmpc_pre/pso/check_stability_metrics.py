"""Check the acceptance semantics with deliberately distinct failure traces."""
import numpy as np
from stability_metrics import summarize
from run_stability_ppo import OUT, files


def main():
    x=np.zeros((500,14)); t=np.arange(500)*.002
    reference=np.ones(500);reference[100:]=0
    x[:200,1]=.2;x[350:,1]=.1
    def metrics():
        return summarize(t,x,reference,np.zeros((500,6)),np.zeros(500),np.ones(500)*.5)
    m=metrics()
    assert abs(m['stops'][0]['stop_s']-.2)<1e-9 and m['stops'][0]['rebound']
    x[250,4]=np.deg2rad(31)
    assert metrics()['failures']==['left_leg_angle']
    x[250,4]=0;x[250,6]=np.deg2rad(31)
    assert metrics()['failures']==['right_leg_angle']
    x[250,6]=0;x[250,8]=np.deg2rad(11)
    assert metrics()['failures']==['pitch_angle']
    reference[150:]=1
    m=metrics()
    assert m['stops'][0]['stop_s'] is None and m['stops'][0]['truncated']
    files.write_json(OUT/'metric_checks.json',dict(passed=True,
        checks=['0.20s dwell from actual reference zero','rebound separately flagged',
                'left angle alone rejects','right angle alone rejects','pitch alone rejects',
                'new command truncates short observation']))
    print('acceptance metric checks PASS')


if __name__=='__main__':main()
