// Experiences "inverses" : on part du design recommande (dedans, S2, coude), on degrade une variable,
// un servo ou une hypothese, et on lit les contraintes cassees et les objectifs, compares a l'optimum.
// Usage : node sim/experiences.js [A|B|C|all]
const fs = require("fs"), path = require("path");
const M = require("./model.js"), C = require("./compare.js"), O = require("./opt.js");
const cfg = require("../audit/config.json"), refs = require("./ref_fronts.json");
const which = process.argv[2] || "all";
const key = "dedans_S2", ref = refs[key], base = ref.choix.coude;
const x0 = { L2: base.L2, L3: base.L3, alpha: base.alpha, beta: base.beta, xi: base.xi };
const cl = (o) => JSON.parse(JSON.stringify(o));
const S0 = M.servoList(cfg), idx = (n) => S0.findIndex((s) => s.name === n);
const combo0 = [base.servo_C1, base.servo_C2, base.servo_C3].map(idx);
const bounds = (() => { const b = cfg.search.bounds, K = C.VARS; return { lo: K.map((k) => b[k][0]), hi: K.map((k) => b[k][1]) }; })();
function run(p, x, combo, sc) {
  const r = M.evaluate(p, x, sc || "S2", { mode: "exact", combo });
  const cons = C.constraints(M, p, r), ob = C.objectives(r, p);
  const bad = cons.filter((c) => c.hard && !c.ok && !c.undef);
  const f = [ob.f1, ob.f2, ob.f3];
  const cmp = f.every(Number.isFinite) ? C.compare(ref, f, x, bounds) : null;
  return { adm: ob.admissible, f1: ob.f1, f2: ob.f2, f3: ob.f3, f2_part: ob.f2partial, f2_notq: ob.f2_notq, f3_notq: ob.f3_notq, nOk: ob.nOk,
    bad: bad.map((c) => ({ id: c.id, val: c.val, lim: c.lim, margin: c.margin, free: !!c.free })), req_free: ob.req_free, dom: cmp ? cmp.dominators : null, regret: cmp ? cmp.regret : null, critical: ob.critical,
    h: r.geo.h, th3: r.pre.th3, ext: Math.max(...r.pre.ext) };
}
function mc(p, x, combo, sc, n, seed) {   // meme procedure que Gr2Opt.optimize : tirages d'incertitudes, taux d'admissibilite
  const R = p.robust, rng = O.mulberry32(seed), u = (a, b) => a + (b - a) * rng();
  let ok = 0;
  for (let i = 0; i < n; i++) {
    const Q = O.perturbed(p, rng), xs = [x.L2 + u(-R.abs_len, R.abs_len), x.L3 + u(-R.abs_len, R.abs_len), x.alpha + u(-R.abs_ang, R.abs_ang), x.beta + u(-R.abs_ang, R.abs_ang), x.xi + u(-R.abs_ang, R.abs_ang)];
    if (O.evalRow(Q, xs, sc, combo)) ok++;
  }
  return ok / n;
}
const out = {};
out.reference = Object.assign({}, run(cl(cfg), x0, combo0), { mc: mc(cl(cfg), x0, combo0, "S2", 100, 99), x: x0, combo: base.servo_C1 + "/" + base.servo_C2 + "/" + base.servo_C3 });
if (which === "A" || which === "all") {
  const D = { L2: [-20, -10, -5, 5, 10, 20], L3: [-20, -10, -5, 5, 10, 20], alpha: [-8, -4, 4, 8, 15, 25], beta: [-20, -10, -5, 5, 10, 15], xi: [-40, -25, -10, 5, 10, 12] };
  out.A = [];
  for (const k of C.VARS) for (const d of D[k]) {
    const x = Object.assign({}, x0, { [k]: x0[k] + d });
    const rr = run(cl(cfg), x, combo0);
    out.A.push(Object.assign({ var: k, delta: d, value: x[k], mc: rr.adm ? mc(cl(cfg), x, combo0, "S2", 100, 99) : null }, rr));
  }
}
if (which === "B" || which === "all") {
  const sw = [["SG90", "S3306", "S-U400"], ["S-U400", "S3306", "S-U400"], ["S3306", "S3306", "S3306"], ["SG90", "S-U400", "S-U400"], ["SG90", "SG90", "S-U400"], ["SG90", "S3306", "SG90"], ["S-U400", "S-U400", "S-U400"], ["SG90", "S3306", "S3306"]];
  out.B = sw.map((c) => Object.assign({ combo: c.join("/") }, run(cl(cfg), x0, c.map(idx))));
}
(async () => {
  if (which === "C" || which === "all") {
    const what = [["référence (config actuelle)", (p) => {}], ["batterie 0,54 → 0,90 kg", (p) => { p.body.battery.m = 0.9; }], ["accélération 0,5 → 1,0 m/s²", (p) => { p.task.accel = 1.0; }],
      ["SF 1,2 → 1,5", (p) => { p.margins.SF = 1.5; }], ["couple utilisable = 70 % du blocage (frac 1,0 → 0,7)", (p) => { p.frac_couple = 0.7; }], ["châssis 200 → 260 mm", (p) => { p.chassis.N_ch = 260; p.chassis.M_ch = 260; }],
      ["masse du bout du bras 0,1 → 0,3 kg", (p) => { p.body.arm_model.m_tip = 0.3; }], ["obstacle 30 → 40 mm", (p) => { p.task.obstacle = 40; }], ["marge NESM cible 2 → 5 mm (ε-contrainte)", (p) => { p.margins.nesm_cible = 5; }]];
    const allowed = [["SG90", "S-U400", "S3306"], ["S3306"], ["S-U400", "S3306"]];
    out.C = [];
    for (const [label, f] of what) {
      const p = cl(cfg); f(p);
      const t = Date.now();
      const r = await O.optimize(p, { mount: "dedans", sc: "S2", allowed, nSobol: 2048, refineRounds: 1, perPoint: 8, nMc: 0, sway_n: p.task.sway_n }, O.syncExec);
      const ch = r.res.choix, k = ch.coude, e = ch.eps_contrainte;
      const row = { label, adm: r.res.admissibles, pareto: r.res.pareto, min_f3: r.res.min_couple_C2, max_f2: r.res.max_nesm, s: Math.round((Date.now() - t) / 1000) };
      if (k) row.coude = { L2: k.L2, L3: k.L3, alpha: k.alpha, beta: k.beta, xi: k.xi, f1: k.f1_masse_pattes, f2: k.f2_nesm, f3: k.f3_couple_C2, servos: k.servo_C1 + "/" + k.servo_C2 + "/" + k.servo_C3 };
      if (e) row.eps = { f1: e.f1_masse_pattes, f2: e.f2_nesm, f3: e.f3_couple_C2, servos: e.servo_C1 + "/" + e.servo_C2 + "/" + e.servo_C3 };
      out.C.push(row); console.error(label, JSON.stringify(row).slice(0, 220));
    }
  }
  if (which === "E") {   // bruit du moteur JS a resolution reduite : meme configuration, autre graine de Sobol
    out.E = [];
    for (const seed of [2, 3]) {
      const p = cl(cfg); p.search.seed = seed;
      const r = await O.optimize(p, { mount: "dedans", sc: "S2", allowed: [["SG90", "S-U400", "S3306"], ["S3306"], ["S-U400", "S3306"]], nSobol: 2048, refineRounds: 1, perPoint: 8, nMc: 0, seed, sway_n: p.task.sway_n }, O.syncExec);
      const k = r.res.choix.coude;
      out.E.push({ seed, adm: r.res.admissibles, pareto: r.res.pareto, min_f3: r.res.min_couple_C2, max_f2: r.res.max_nesm, coude: k && { L2: k.L2, L3: k.L3, f1: k.f1_masse_pattes, f2: k.f2_nesm, f3: k.f3_couple_C2, servos: k.servo_C1 + "/" + k.servo_C2 + "/" + k.servo_C3 } });
      console.error("seed", seed, JSON.stringify(out.E[out.E.length - 1]).slice(0, 200));
    }
  }
  fs.writeFileSync(path.join(__dirname, "../audit/experiences" + (which === "all" ? "" : "_" + which) + ".json"), JSON.stringify(out));
  console.log("ok", which);
})();
