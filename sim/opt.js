/* Gr2 - portage JavaScript du PIPELINE D'OPTIMISATION complet (gr2/explore.py,
 * gr2/pipeline.py) : echantillonnage de Sobol, pre-filtre, evaluation exacte de
 * chaque combinaison de servos, raffinement local, Pareto 3D (skyline), couches
 * de non-dominance, Monte-Carlo, front robuste, choix (coude, epsilon-contrainte).
 * Il s'appuie sur model.js (evaluate), deja valide contre le Python.
 * Les etapes deterministes (Pareto, couches, coude, epsilon) sont verifiees
 * contre Python par sim/validate_opt.js ; les etapes aleatoires (Sobol brouille,
 * raffinement, Monte-Carlo) utilisent le meme algorithme mais un autre generateur.
 */
(function (root) {
  "use strict";
  const M = typeof require !== "undefined" && typeof module !== "undefined" ? require("./model.js") : root.Gr2Model;
  const VARS = ["L2", "L3", "alpha", "beta", "xi"];
  const clone = (o) => JSON.parse(JSON.stringify(o));

  // ------------------------------------------------------------------ RNG
  function mulberry32(a) {
    return function () {
      a |= 0; a = (a + 0x6d2b79f5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  const uni = (rng, a, b) => a + (b - a) * rng();

  // ---------------------------------------------------------------- Sobol
  // Directions de Joe & Kuo (new-joe-kuo-6.21201), dimensions 1 a 5, brouillage
  // par decalage numerique aleatoire (XOR), comme scipy.stats.qmc.Sobol(scramble=True) en esprit.
  const JK = [null, { s: 1, a: 0, m: [1] }, { s: 2, a: 1, m: [1, 3] }, { s: 3, a: 1, m: [1, 3, 1] }, { s: 3, a: 2, m: [1, 1, 7] }];
  function directions(d) {
    const V = new Array(32);
    if (d === 0) { for (let k = 0; k < 32; k++) V[k] = (1 << (31 - k)) >>> 0; return V; }
    const { s, a, m } = JK[d];
    for (let k = 0; k < s; k++) V[k] = (m[k] << (31 - k)) >>> 0;
    for (let k = s; k < 32; k++) {
      let v = V[k - s] ^ (V[k - s] >>> s);
      for (let i = 1; i < s; i++) if ((a >> (s - 1 - i)) & 1) v ^= V[k - i];
      V[k] = v >>> 0;
    }
    return V;
  }
  function sobol(n, dim, seed) {
    const rng = mulberry32(seed || 1), Vs = [], shift = [];
    for (let d = 0; d < dim; d++) { Vs.push(directions(d)); shift.push((rng() * 4294967296) >>> 0); }
    const x = new Array(dim).fill(0), out = [];
    for (let i = 0; i < n; i++) {
      if (i > 0) { let c = 0, v = i; while (v & 1) { v >>= 1; c++; } for (let d = 0; d < dim; d++) x[d] = (x[d] ^ Vs[d][c]) >>> 0; }
      out.push(x.map((q, d) => ((q ^ shift[d]) >>> 0) / 4294967296));
    }
    return out;
  }
  function bounds(p) { const b = p.search.bounds; return [VARS.map((k) => b[k][0]), VARS.map((k) => b[k][1])]; }
  function sobolDesigns(p, n, seed) {
    const [lo, hi] = bounds(p);
    return sobol(n, 5, seed).map((u) => u.map((v, k) => lo[k] + v * (hi[k] - lo[k])));
  }

  // --------------------------------------------------------- une evaluation
  /* Retourne une ligne (memes colonnes que gr2/evaluate.py) ou null si le
   * design (pre-filtre, 14 phases, butees, masse) n'est pas admissible avec ce combo. */
  function evalRow(p, xv, sc, combo) {
    const x = { L2: xv[0], L3: xv[1], alpha: xv[2], beta: xv[3], xi: xv[4] };
    const geo = M.geometry(p, x);
    if (!M.prefilter(p, geo).ok) return null;
    const r = M.evaluate(p, x, sc, { mode: "exact", combo });
    const s = r.servo;
    if (!s.admissible) return null;
    const S = M.servoList(p);
    const ssm = Math.min(...s.phases.filter((q) => q.ok).map((q) => q.ssmc));
    return {
      L1: p.leg.L1, L2: x.L2, L3: x.L3, alpha: x.alpha, beta: x.beta, xi: x.xi, D: geo.D0, H2: geo.H2, h: geo.h,
      servo_C1: S[combo[0]].name, servo_C2: S[combo[1]].name, servo_C3: S[combo[2]].name,
      f1_masse_pattes: s.f1, f2_nesm: s.f2, f3_couple_C2: s.f3, ssm_corr_min: ssm,
      frac_bloc_C1: s.fracBloc[0], frac_bloc_C2: s.fracBloc[1], frac_bloc_C3: s.fracBloc[2],
      u_C1: s.u[0], u_C2: s.u[1], u_C3: s.u[2],
      tau_C1: s.tauMaxBest[0], tau_C2: s.tauMaxBest[1], tau_C3: s.tauMaxBest[2],
      tau_min_C1: s.tmin[0], tau_min_C2: s.tmin[1], tau_min_C3: s.tmin[2],
      span_C1: s.span[0], span_C2: s.span[1], span_C3: s.span[2],
      M_total: s.Mc, phase_critique: s.critical, taux_reussite: null,
    };
  }
  function evalBatch(items, p, sc) { return items.map((it) => evalRow(it.p || p, it.x, sc, it.combo)); }

  // ----------------------------------------------------------------- Pareto
  const F = (r) => [r.f1_masse_pattes, -r.f2_nesm, r.f3_couple_C2];
  function paretoMask(Fm) {
    const order = Fm.map((_, i) => i).sort((a, b) => Fm[a][0] - Fm[b][0] || Fm[a][1] - Fm[b][1] || Fm[a][2] - Fm[b][2] || a - b);
    const keep = new Array(Fm.length).fill(false), front = [], e = 1e-12;
    for (const i of order) {
      const f = Fm[i]; let skip = false;
      for (const q of front) {
        let le = true, lt = false, eq = true;
        for (let k = 0; k < 3; k++) { if (!(q[k] <= f[k] + e)) le = false; if (q[k] < f[k] - e) lt = true; if (!(Math.abs(q[k] - f[k]) <= e)) eq = false; }
        if ((le && lt) || eq) { skip = true; break; }
      }
      if (skip) continue;
      keep[i] = true; front.push(f);
    }
    return keep;
  }
  function pareto(rows) {
    if (!rows.length) return [];
    const m = paretoMask(rows.map(F));
    return rows.filter((_, i) => m[i]).sort((a, b) => a.f3_couple_C2 - b.f3_couple_C2);
  }
  function paretoLayers(rows, k, cap) {
    let rest = rows.slice(); const layers = [];
    for (let l = 0; l < k; l++) {
      if (!rest.length) break;
      const m = paretoMask(rest.map(F));
      layers.push(rest.filter((_, i) => m[i])); rest = rest.filter((_, i) => !m[i]);
    }
    let out = [].concat(...layers);
    if (out.length > cap) {
      out = out.slice().sort((a, b) => a.f3_couple_C2 - b.f3_couple_C2);
      out = Array.from({ length: cap }, (_, i) => out[Math.floor((i * (out.length - 1)) / (cap - 1))]);
    }
    return out;
  }
  function kneePoint(front) {
    if (!front.length) return null;
    if (front.length < 3) return 0;
    const Fm = front.map(F), mn = [0, 1, 2].map((k) => Math.min(...Fm.map((f) => f[k]))), mx = [0, 1, 2].map((k) => Math.max(...Fm.map((f) => f[k])));
    let best = 0, bv = Infinity;
    Fm.forEach((f, i) => { const d = Math.hypot(...[0, 1, 2].map((k) => (f[k] - mn[k]) / Math.max(mx[k] - mn[k], 1e-12))); if (d < bv) { bv = d; best = i; } });
    return best;
  }
  function epsChoice(front, target) {
    const ok = front.map((r, i) => [r, i]).filter(([r]) => r.f2_nesm >= target);
    if (!ok.length) return null;
    ok.sort((a, b) => a[0].f3_couple_C2 - b[0].f3_couple_C2 || a[0].f1_masse_pattes - b[0].f1_masse_pattes);
    return ok[0][1];
  }

  // -------------------------------------------------------- robustesse (MC)
  function perturbed(p, rng) {
    const R = p.robust, Q = clone(p), b = Q.body;
    for (const key of ["chassis", "battery"]) {
      b[key].m *= 1 + uni(rng, -R.rel_mass, R.rel_mass);
      b[key].x += uni(rng, -R.abs_cdg, R.abs_cdg); b[key].y += uni(rng, -R.abs_cdg, R.abs_cdg); b[key].z_rel += uni(rng, -R.abs_cdg, R.abs_cdg);
    }
    const am = b.arm_model;
    for (const key of ["m_base", "m_elbow", "m_tip"]) am[key] *= 1 + uni(rng, -R.rel_mass, R.rel_mass);
    am.mount_from_front += uni(rng, -R.abs_cdg, R.abs_cdg); am.mount_y += uni(rng, -R.abs_cdg, R.abs_cdg); am.mount_z_rel += uni(rng, -R.abs_cdg, R.abs_cdg);
    for (const s of Q.servos) { s.C_stall *= 1 - uni(rng, 0, R.rel_torque); s.mass *= 1 + uni(rng, -R.rel_mass, R.rel_mass); }
    Q.leg.seg_lin_mass *= 1 + uni(rng, -R.rel_mass, R.rel_mass);
    Q.leg.L1 += uni(rng, -R.abs_len, R.abs_len);
    Q.chassis.e += uni(rng, -R.abs_len, R.abs_len);
    return Q;
  }

  /* Pipeline complet. exec(items, p, sc) -> Promise<rows[]> : executeur (worker pool
   * dans le navigateur, boucle synchrone sous Node). hooks.progress(f, msg), hooks.stop(). */
  async function optimize(cfg, o, exec, hooks) {
    hooks = hooks || {};
    const t0 = Date.now(), log = hooks.log || (() => {}), prog = hooks.progress || (() => {});
    const p = clone(cfg); p.chassis.mount = o.mount;
    if (o.sway_n) p.task.sway_n = o.sway_n;
    if (o.nSobol) p.search.n_sobol = o.nSobol;
    if (o.refineRounds !== undefined) p.search.refine_rounds = o.refineRounds;
    if (o.perPoint) p.search.refine_per_point = o.perPoint;
    if (o.nMc !== undefined) p.robust.n_mc = o.nMc;
    const sc = o.sc, S = M.servoList(p), names = new Map(S.map((s, i) => [s.name, i]));
    let combos = o.combos;
    if (!combos) {
      const lists = o.allowed.map((a) => a.map((n) => names.get(n)).filter((i) => i !== undefined));
      combos = [];
      for (const a of lists[0]) for (const b of lists[1]) for (const c of lists[2]) {
        const cc = [a, b, c]; let ok = true;
        for (const i of new Set(cc)) { const st = S[i].stock, k = cc.filter((v) => v === i).length; if (st !== null && st !== undefined && 4 * k > st) ok = false; }
        if (ok) combos.push(cc);
      }
    }
    if (!combos.length) throw new Error("aucune combinaison de servos compatible avec le stock");
    const [lo, hi] = bounds(p), rng = mulberry32((o.seed || 7) * 7919 + 13);
    const X = o.X || sobolDesigns(p, p.search.n_sobol, p.search.seed);
    const total = combos.length + 2; let done = 0;
    const parts = [], stats = {}, maxRef = o.maxRefine || 150;
    for (const cb of combos) {
      if (hooks.stop && hooks.stop()) break;
      const key = cb.map((i) => S[i].name).join("/");
      let rows = (await exec(X.map((x) => ({ x, combo: cb })), p, sc)).filter(Boolean);
      const geom = new Set(rows.map((r) => [r.L2, r.L3, r.alpha, r.beta, r.xi].map((v) => v.toFixed(6)).join(",")));
      for (let rd = 0; rd < p.search.refine_rounds; rd++) {
        const fr = pareto(rows); if (!fr.length) break;
        const seen = new Set(), X0 = [];
        for (const r of fr) { const k = [r.L2, r.L3, r.alpha, r.beta, r.xi].join(","); if (!seen.has(k)) { seen.add(k); X0.push([r.L2, r.L3, r.alpha, r.beta, r.xi]); } }
        const X0c = X0.slice(0, maxRef), n = p.search.refine_per_point, Xr = [];
        for (const x0 of X0c) for (let q = 0; q < n; q++) Xr.push(x0.map((v, k) => Math.min(hi[k], Math.max(lo[k], v + uni(rng, -1, 1) * 0.05 * (hi[k] - lo[k])))));
        rows = rows.concat((await exec(Xr.map((x) => ({ x, combo: cb })), p, sc)).filter(Boolean));
      }
      stats[key] = { geometries_admissibles: geom.size, apres_raffinement: rows.length };
      parts.push(rows); done++; prog(done / total, `${key} : ${rows.length} designs admissibles`); log(`${key} : ${rows.length}`);
    }
    const df = [].concat(...parts), front = pareto(df), cand = paretoLayers(df, 6, 60);
    // Monte-Carlo
    const R = p.robust, passes = new Array(cand.length).fill(0), rmc = mulberry32((o.seed || 7) * 104729 + 11);
    const candCombo = cand.map((r) => [names.get(r.servo_C1), names.get(r.servo_C2), names.get(r.servo_C3)]);
    for (let it = 0; it < R.n_mc && cand.length; it++) {
      if (hooks.stop && hooks.stop()) break;
      const Q = perturbed(p, rmc);
      const items = cand.map((r, i) => ({ x: [r.L2 + uni(rmc, -R.abs_len, R.abs_len), r.L3 + uni(rmc, -R.abs_len, R.abs_len), r.alpha + uni(rmc, -R.abs_ang, R.abs_ang), r.beta + uni(rmc, -R.abs_ang, R.abs_ang), r.xi + uni(rmc, -R.abs_ang, R.abs_ang)], combo: candCombo[i] }));
      (await exec(items, Q, sc)).forEach((r, i) => { if (r) passes[i]++; });
      if ((it + 1) % 5 === 0) prog((combos.length + (it + 1) / R.n_mc) / total, `Monte-Carlo ${it + 1}/${R.n_mc}`);
    }
    const rob = cand.map((r, i) => Object.assign({}, r, { taux_reussite: R.n_mc > 0 ? passes[i] / R.n_mc : 1 }));
    const robOk = rob.filter((r) => r.taux_reussite >= R.pass_rate), robFront = pareto(robOk);
    const choix = {};
    if (robFront.length) {
      choix.coude = robFront[kneePoint(robFront)];
      const ec = epsChoice(robFront, p.margins.nesm_cible ?? 2);
      if (ec !== null) choix.eps_contrainte = robFront[ec];
      choix.couple_min = robFront.reduce((a, b) => (b.f3_couple_C2 < a.f3_couple_C2 ? b : a));
    } else if (rob.length) choix.meilleur_non_robuste = rob.slice().sort((a, b) => b.taux_reussite - a.taux_reussite || a.f3_couple_C2 - b.f3_couple_C2)[0];
    for (const r of Object.values(choix)) { const x = VARS.map((k) => r[k]); r.bornes_actives = VARS.filter((v, k) => Math.abs(x[k] - lo[k]) < 0.5 || Math.abs(x[k] - hi[k]) < 0.5); }
    prog(1, "terminé");
    const res = {
      stats, admissibles: df.length, pareto: front.length, candidats: cand.length, robustes: robOk.length, pareto_robuste: robFront.length,
      min_couple_C2: df.length ? Math.min(...df.map((r) => r.f3_couple_C2)) : null, max_nesm: df.length ? Math.max(...df.map((r) => r.f2_nesm)) : null,
      choix, duree_s: Math.round((Date.now() - t0) / 1000), combos: combos.map((c) => c.map((i) => S[i].name)),
    };
    return { res, adm: df.length > 3000 ? Array.from({ length: 3000 }, (_, i) => df[Math.floor((i * (df.length - 1)) / 2999)]) : df, front, cand: rob, rob_front: robFront };
  }

  // ----------------------------------------- pre-selection indicative des servos
  async function screenServos(cfg, sc, mount, exec, n) {
    const p = clone(cfg); p.chassis.mount = mount;
    const mmin = Math.min(...p.servos.map((s) => s.mass));
    p.servos.push({ name: "ILLIMITE", C_stall: 100.0, mass: mmin, range: 360.0, stock: null });
    const idx = p.servos.length - 1, X = sobolDesigns(p, n || 512, 5);
    const rows = (await exec(X.map((x) => ({ x, combo: [idx, idx, idx] })), p, sc)).filter(Boolean);
    if (!rows.length) return { ok: false, minima: [null, null, null], allowed: [[], [], []], n_adm: 0 };
    const mins = [Math.min(...rows.map((r) => r.tau_min_C1)), Math.min(...rows.map((r) => r.f3_couple_C2)), Math.min(...rows.map((r) => r.tau_min_C3))];
    const SF = cfg.margins.SF, fr = cfg.frac_couple ?? 1;
    const allowed = [0, 1, 2].map((j) => cfg.servos.filter((s) => (s.stock === null || s.stock === undefined || s.stock >= 4) && (fr * s.C_stall) / SF >= 0.98 * mins[j]).map((s) => s.name));
    return { ok: true, minima: mins, allowed, n_adm: rows.length };
  }

  // executeur synchrone (Node, tests)
  const syncExec = async (items, p, sc) => evalBatch(items, p, sc);

  const api = { VARS, mulberry32, sobol, sobolDesigns, bounds, evalRow, evalBatch, paretoMask, pareto, paretoLayers, kneePoint, epsChoice, perturbed, optimize, screenServos, syncExec };
  if (typeof module !== "undefined" && module.exports) module.exports = api; else root.Gr2Opt = api;
})(typeof self !== "undefined" ? self : this);
