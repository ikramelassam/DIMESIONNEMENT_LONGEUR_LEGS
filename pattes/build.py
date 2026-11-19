import json, yaml, os
H=os.path.dirname(os.path.abspath(__file__)); G=os.path.dirname(H)
cfg=yaml.safe_load(open(G+'/config.yaml',encoding='utf-8')); cfg['gabarit']=400.0
rf=json.load(open(G+'/sim/ref_fronts.json'))
pre=[]
for key,lab in [('dedans_S2','Dedans, pire cas S2 (coude)'),('dedans_S1','Dedans, nominal S1 (coude)'),('dehors_S2','Dehors, pire cas S2 (coude)'),('dehors_S1','Dehors, nominal S1 (coude)')]:
    c=rf[key]['choix']['coude']; m,s=key.split('_')
    pre.append(dict(label=lab,mount=m,sc=s,x={k:round(c[k],2) for k in ['L2','L3','alpha','beta','xi']},servos=[c['servo_C1'],c['servo_C2'],c['servo_C3']]))
t=open(H+'/tpl.html',encoding='utf-8').read()
model=open(G+'/sim/model.js',encoding='utf-8').read(); assert '</script' not in model
out=t.replace('/*MODEL*/',model).replace('/*CONFIG*/',json.dumps(cfg,ensure_ascii=False)).replace('/*PRESETS*/',json.dumps(pre,ensure_ascii=False))
open(H+'/pattes_gr2.html','w',encoding='utf-8').write(out); print(len(out)//1024,'Ko')
