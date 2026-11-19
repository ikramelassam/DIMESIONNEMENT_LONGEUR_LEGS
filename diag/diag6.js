const M=require('/home/claude/gr2_dim/sim/model.js');
const cfg=require('/home/claude/gr2_dim/audit/config.json'); cfg.gabarit=400;
const base={L2:60.08,L3:71.92,alpha:10.55,beta:99.76};
for(const xi of [45,77.32]){const p=JSON.parse(JSON.stringify(cfg));p.chassis.mount='dedans';const ix=n=>p.servos.findIndex(s=>s.name===n);
 const r=M.evaluate(p,{...base,xi},'S2',{mode:'exact',combo:['S3306','S3306','S3306'].map(ix)});const s=r.servo;
 console.log('\nxi',xi,'phase critique',s.critical,'f2',s.f2.toFixed(2),'| NESM par phase:',s.phases.map(q=>q.name+'='+q.nesm.toFixed(1)+'('+q.sway.map(v=>v.toFixed(0)).join(';')+')').join(' '));
 console.log(' couple requis min (tmin) C1/C2/C3',s.tmin.map(v=>v.toFixed(3)).join('/'),'; phases les plus chargées en C3:',s.phases.map(q=>[q.name,q.tau?q.tau[2]:0]).sort((a,b)=>b[1]-a[1]).slice(0,3).map(a=>a[0]+' '+a[1].toFixed(2)).join(', '));
 // phase critique : detail etats
 const pi=s.phases.findIndex(q=>q.name===s.critical),ph=s.phases[pi];
 r.detail[pi].forEach((d,j)=>{const v=M.stateView(p,r,pi,j,ph.best);const pm=M.polygonMargins(v.feet,v.Gc,v.hxy,v.zmax);console.log('  ',d.label,'| NESM',pm.nesm.toFixed(2),'SSM',pm.ssm.toFixed(1),'| arêtes',pm.edges.map(e=>e.toFixed(1)).join(', '),'| hxy',v.hxy.map(a=>a.toFixed(1)).join(','));});
}
