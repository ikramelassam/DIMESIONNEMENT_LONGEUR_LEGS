// Compare sim/model.js au programme Python (sim/ref.json). Usage : node sim/validate.js
const M = require("./model.js");
const ref = require("./ref.json");
let nExactAdm = 0;
let maxErr = { nesm: 0, ssm: 0, k: 0, swA2: 0, eps: 0, row: 0 }, geoMismatch = 0, admMismatch = 0, nCmp = 0, nCombo = 0;
const rel = (a, b) => Math.abs(a - b) / Math.max(1, Math.abs(b));
for (const c of ref.cases) {
  const p = JSON.parse(JSON.stringify(ref.config)); p.chassis.mount = c.mount;
  const r = M.evaluate(p, c.x, c.scenario, { mode: "programme" });
  maxErr.eps = Math.max(maxErr.eps, rel(r.epsLegsXY, c.eps_legs));
  for (let pi = 0; pi < c.nesm.length; pi++) for (let s = 0; s < c.nesm[pi].length; s++) {
    nCmp++;
    if (r.geoOk[pi][s] !== c.geo_ok[pi][s]) geoMismatch++;
    maxErr.nesm = Math.max(maxErr.nesm, rel(r.nesm[pi][s], c.nesm[pi][s]));
    maxErr.ssm = Math.max(maxErr.ssm, rel(r.ssmc[pi][s], c.ssm_c[pi][s]));
    maxErr.swA2 = Math.max(maxErr.swA2, rel(r.swA2[pi][s], c.swA2[pi][s]));
    for (let j = 0; j < 3; j++) maxErr.k = Math.max(maxErr.k, rel(r.k[pi][s][j], c.k[pi][s][j]));
  }
  // mode exact, combinaison mixte S-U400 / S3306 / S-U400
  {
    const S0 = M.servoList(p); const cb = c.exact.combo.map((n) => S0.findIndex((s) => s.name === n));
    const rx = M.evaluate(p, c.x, c.scenario, { mode: "exact", combo: cb });
    for (let pi = 0; pi < c.exact.nesm.length; pi++) for (let s = 0; s < c.exact.nesm[pi].length; s++) {
      maxErr.exact = Math.max(maxErr.exact || 0, rel(rx.nesm[pi][s], c.exact.nesm[pi][s]));
      for (let j = 0; j < 3; j++) maxErr.exact = Math.max(maxErr.exact, rel(rx.k[pi][s][j], c.exact.k[pi][s][j]));
    }
    if ((c.exact.rows.length > 0) !== rx.servo.admissible) { admMismatch++; console.log("exact : admissibilite differente"); }
    if (c.exact.rows.length) { const q = c.exact.rows[0]; maxErr.exact = Math.max(maxErr.exact, rel(rx.servo.f2, q.f2_nesm), rel(rx.servo.f3, q.f3_couple_C2)); nExactAdm = (nExactAdm || 0) + 1; }
  }
  // admissibilite et sorties, pour toutes les combinaisons de servos autorisees
  const S = M.servoList(p);
  for (const cb of M.combos(p)) {
    nCombo++;
    const o = M.resolve(p, r, cb);
    const row = c.rows.find((q) => q.servo_C1 === S[cb[0]].name && q.servo_C2 === S[cb[1]].name && q.servo_C3 === S[cb[2]].name);
    if (!!row !== o.admissible) { admMismatch++; console.log("admissibilite differente", c.mount, c.scenario, cb, o.admissible); }
    if (row) {
      const pairs = [[o.f1, row.f1_masse_pattes], [o.f2, row.f2_nesm], [o.f3, row.f3_couple_C2], [o.tauMaxBest[1], row.tau_C2],
        [o.tauMaxBest[2], row.tau_C3], [o.tmin[2], row.tau_min_C3], [o.span[1], row.span_C2], [o.Mc, row.M_total], [o.u[1], row.u_C2]];
      for (const [a, b] of pairs) maxErr.row = Math.max(maxErr.row, rel(a, b));
      if (o.critical !== row.phase_critique) console.log("phase critique differente", o.critical, row.phase_critique);
    }
  }
}
const summary = { cas: ref.cases.length, valeurs_comparees: nCmp, combinaisons: nCombo, geo_ok_differents: geoMismatch, admissibilite_differente: admMismatch, exact_admissibles_compares: nExactAdm, ecart_relatif_max: maxErr };
console.log(JSON.stringify(summary, null, 1));
require("fs").writeFileSync(__dirname + "/validation.json", JSON.stringify(summary));
