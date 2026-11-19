// Valide sim/opt.js contre Python : (1) evalRow = lignes reelles de results_exact ; (2) Pareto, couches, coude, epsilon.
const O = require("./opt.js"), ref = require("./ref_opt.json");
const rel = (a, b) => Math.abs(a - b) / Math.max(1, Math.abs(b));
let ties = 0, maxErr = 0, nRow = 0, admDiff = 0, par = { ok: 0, n: 0 }, out = {};
const KEYS = ["f1_masse_pattes", "f2_nesm", "f3_couple_C2", "tau_C1", "tau_C2", "tau_C3", "tau_min_C1", "tau_min_C2", "tau_min_C3", "span_C1", "span_C2", "span_C3", "M_total", "u_C2", "frac_bloc_C2", "ssm_corr_min", "D", "h"];
for (const c of ref.cases) {
  const p = JSON.parse(JSON.stringify(ref.config)); p.chassis.mount = c.mount;
  const names = p.servos.map((s) => s.name);
  // (1) 60 lignes reelles
  for (let k = 0; k < c.rows.length; k += Math.floor(c.rows.length / 60)) {
    const q = c.rows[k], cb = [q.servo_C1, q.servo_C2, q.servo_C3].map((n) => names.indexOf(n));
    const r = O.evalRow(p, [q.L2, q.L3, q.alpha, q.beta, q.xi], c.sc, cb); nRow++;
    if (!r) { admDiff++; continue; }
    for (const key of KEYS) maxErr = Math.max(maxErr, rel(r[key], q[key]));
    if (r.phase_critique !== q.phase_critique) {   // verifie que c'est un ex aequo (pattes symetriques) et non un ecart
      const M = require("./model.js"), full = M.evaluate(p, { L2: q.L2, L3: q.L3, alpha: q.alpha, beta: q.beta, xi: q.xi }, c.sc, { mode: "exact", combo: cb });
      const nes = (nm) => full.servo.phases.find((z) => z.name === nm).nesm;
      if (Math.abs(nes(r.phase_critique) - nes(q.phase_critique)) < 1e-6) ties++; else { admDiff++; console.log("phase critique differente", r.phase_critique, q.phase_critique); }
    }
  }
  // (2) Pareto & co sur les 700 lignes
  const rows = c.rows.map((r, i) => Object.assign({ _i: i }, r));
  const fr = O.pareto(rows), ly = O.paretoLayers(rows, 6, 60);
  const same = (a, b) => a.length === b.length && a.every((v, i) => v === b[i]);
  const frIds = fr.map((r) => r._i), lyIds = ly.map((r) => r._i);
  const ki = O.kneePoint(fr), ei = O.epsChoice(fr, 2.0);
  out[c.case] = { pareto_identique: same(frIds, c.pareto), pareto_n: fr.length, couches_identiques: same(lyIds.slice().sort((a, b) => a - b), c.layers.slice().sort((a, b) => a - b)), couches_n: ly.length,
    coude_identique: fr[ki]._i === c.knee, eps_identique: (ei === null ? null : fr[ei]._i) === c.eps };
}
const summary = { lignes_comparees: nRow, ecart_relatif_max: maxErr, admissibilite_ou_phase_differente: admDiff, phases_critiques_ex_aequo: ties, optimiseur: out };
console.log(JSON.stringify(summary, null, 1));
require("fs").writeFileSync(__dirname + "/validation_opt.json", JSON.stringify(summary));
